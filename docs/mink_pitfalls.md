# mink / MuJoCo IK の地雷集（manekko 向け）

海外プログラマのサイト・mink公式・GitHub issue・MuJoCoドキュメント・差分IKの論文から収集。
manekko の構成（free root + ball/hinge、10点 position-only FrameTask + PostureTask、daqp）に
特に効くものを上に置く。出典は末尾。

## ★最重要: A-pose は特異姿勢（straight-limb singularity）

差分IK（ヤコビアン線形化）は**特異点と関節限界の近傍で破綻**しやすい
（"locally linearized updates often become trapped near joint limits and singularities" — MIRROR論文）。
manekko の素体 A-pose は**肘・膝がほぼ伸び切っている＝特異**。ここで FrameTask の目標が
腕方向にずれると、ヤコビアンが退化して関節速度が暴れる/肘が裏返る。

対策:
- `lm_damping`（タスク毎のLevenberg-Marquardt減衰）を効かせる。G1例は全タスク `lm_damping=1.0`。manekko も 1.0 採用済み。
- `solve_ik(..., damping=1e-1)` のグローバル減衰を維持（G1と同値）。下げ過ぎると特異点で発散。
- **キャリブレーションの素体に微小な肘膝の曲げ（数度）を持たせる**か、PostureTaskのターゲットを
  完全伸展でなく僅かに曲げた姿勢にすると、特異点から離れて安定する。要実験。
- hinge（肘膝）の回転軸が A-pose では `cross(parent_dir, self_dir)≈0` になり
  rig.py はフォールバック軸を使っている。**実トラッカーで肘膝が変な向きに曲がるならここが原因**。
  軸の符号・方向、または関節限界（下記）で矯正する。

## ★関節限界が無い → 過伸展・裏返り

現状の MJCF は hinge/ball に range 未設定（事実上無制限）。肘膝が逆に曲がる、肩がねじれる等が起きうる。
- G1公式例は `limits = [mink.ConfigurationLimit(model), collision_avoidance_limit]` を使用。
- 対策案: MJCF の hinge に `range`（肘 0〜150°, 膝 0〜150° 等）を入れる、または
  `mink.ConfigurationLimit(model)` を tasks と別に limits で渡す。
- 注意: `ConfigurationLimit` は過去に不具合（issue #109, 〜2025/10で修正）、
  velocity limit と IK積分の不整合（issue #137, 2026/04修正）。**mink>=1.1.1 を維持**（同梱版でOK）。

## qpos と nv のインデックスは別物

- ball joint: qpos 4個（クォータニオン w,x,y,z）/ dof 3個。free joint: qpos 7（pos3+quat4）/ dof 6。
- **qpos を dofインデックスで触らない**。`model.jnt_qposadr[jid]` / `model.jnt_dofadr[jid]` を使う。
  （rig/solver の現コードは jnt_qposadr 経由で正しい。）

## クォータニオン正規化

- ball/free を速度で積分すると quat が単位長から外れる。`configuration.integrate_inplace` は
  内部で `mj_integratePos` を使い正規化するので**通常は安全**。
- **自前で qpos を代入する場合**は `mujoco.mj_normalizeQuat` を呼ぶ。quat を線形補間しない。

## solve_ik は「速度」を返す（姿勢ではない）

- 返り値 `vel`(nv) を `configuration.integrate_inplace(vel, dt)` で積分して初めて姿勢が進む。
  `dt` がステップ幅。manekko は実装済み。

## NoSolutionFound 例外

- QPが不可解（限界の矛盾等）だと `solve_ik` が `NoSolutionFound` を投げる（新しめの版で追加）。
- **ライブ modal operator では try/except で捕捉し、失敗フレームは前ポーズ保持**。落とさない。

## ★実観測: ヌル空間ドリフト（周期運動で累積するねじれ） — posture_cost で解決

**症状（2026-05-30 実機）**: 位置のみIKでライブ駆動すると、最初はまともだが、足踏み等の
**周期運動を繰り返すと体（脊椎・腕・ルート）がだんだん同方向に巻き上がる**。逆向きに動かすと
**巻き戻る（可逆）**。腕は肘160°まで巻いた例あり。
**原因**: 差分IKは毎フレーム速度を積分する経路依存の方式。**位置のみ目標**だと体のひねり等の
冗長自由度（ヌル空間）に絶対基準が無く、入力経路に依存して累積する（null-space wind-up / ratcheting）。
**解決**: **PostureTask の cost を上げて rest(A-pose) へ毎フレーム引き戻す絶対基準を与える**。
`posture_cost` を **1e-2 → 1e-1** で観測ドリフトが止まり、追従も許容範囲（`solver.py` 既定値に採用）。
さらに必要なら関節限界（下記）併用。

## PostureTask は冗長自由度の安定化に必須

- 49dof に対しタスク拘束が多いとはいえ、肘の円運動・脊椎配分などヌル空間方向は
  PostureTask が無いと漂流/ジッタする。**必ず入れる**（実装済み）。
- コスト目安: G1公式は `cost=1e-1`。**manekko も 1e-1 を採用**（上記ドリフトを解消）。
  **上げると安定＆人間らしいが追従が緩む**。さらに必要なら 0.3〜0.5 へ。微調整は今後の課題。

## ComTask（重心）

- G1全身例は `mink.ComTask(cost=10.0)` で重心を拘束しバランスを取る。
- manekko は物理立位でなく hip トラッカーで腰を直接拘束するので必須ではないが、
  全身が不自然に漂う場合は軽い ComTask 追加を検討。

## solver / dt / 反復のトレードオフ

- 公式 daqp + damping=1e-1 を踏襲（manekko 同じ）。
- G1は 200Hz・1ステップ1solve（高レート＋ウォームスタート前提）。manekko は 24〜60fps なので
  **1フレーム複数反復**で補う（検証で 6反復×数フレームのウォームスタートで ~7mm 収束を確認）。

## frame_type="body" の目標位置

- body フレームはボディ原点。トラッカー実装置がボディ原点とずれる分は
  **A-poseキャリブレーションのオフセットで吸収**する設計（site を切る案もある）。

## 初期化

- 最初の solve 前に `configuration.update(qpos0)` か `update_from_keyframe(...)` を必ず呼ぶ。
  未更新だとヤコビアンが古く誤動作。

---

## 出典
- [kevinzakka/mink (GitHub)](https://github.com/kevinzakka/mink)
- [mink humanoid_g1 例](https://github.com/kevinzakka/mink/blob/main/examples/humanoid_g1.py)
- [mink CHANGELOG（NoSolutionFound 等）](https://github.com/kevinzakka/mink/blob/main/CHANGELOG.md)
- [mink docs](https://kevinzakka.github.io/mink/)
- [mink issues #109 / #135 / #137](https://github.com/kevinzakka/mink/issues)
- [MuJoCo modeling（quaternion / mj_normalizeQuat）](https://mujoco.readthedocs.io/en/latest/modeling.html)
- [MIRROR: real-time retargeting w/ differential IK（特異点・限界での失敗）](https://arxiv.org/pdf/2603.23995)
- [HL-IK: human-like elbow prior for humanoid arms](https://arxiv.org/pdf/2509.20263)
- [Damped Least-Squares IK レビュー](https://stephanniec.github.io/stepholio/files/leastsqrinvkin.pdf)
