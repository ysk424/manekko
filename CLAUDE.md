# Manekko — Working Notes (read this first)

Handoff log for future Claude Code sessions. Update when phases close.
Communication is Japanese. Owner: `azoo` / `ysk424` (ysk424@hotmail.com).

---

## ⚠️ START HERE — 2026-05-31（v0.1.5 完成・公開済み・これを最初に読む）

**いまの状態**: `dist/manekko-0.1.5.zip` ＝ **完成品。コミット/プッシュ済み、GitHub を public 化済み**
（参照実装として）。**ここで一区切り**。次にこの Code を起動するのは**利用中に見つけたバグの修正**のとき
（バグが無ければ当分アクセスされない＝休眠）。**フェイス(iPhone ARKit)は別プロジェクトで本リポ対象外**。
カスタムmocap方針＝不具合は都度実験対応。

**バグ修正で来た未来の自分へ**: まず下の「本番パスの最重要事実」を読む（ライブは `ops._read_valid` で
reader スレッドではない）。各サブシステムの詳細 docs: 回転=`docs/rotation_notes.md`、手/指/入力/wheel=
`docs/hands_and_input.md`、IK地雷=`docs/mink_pitfalls.md`、ライブ/座標=`docs/live_driving_notes.md`。

### v0.1.5 で何ができるか（実機検証済み）
- SteamVR の VIVE Tracker＋コントローラ → mink 差分IK → CCキャラを全身ライブ駆動＋Action録画。
- **IKターゲット=7点（位置＋回転）**: hip, head, hand_l/r(=手首), foot_l/r, chest。
- 回転: 足を伸ばしたまま捻れば追従、T/Aポーズで腕を捻れる（`docs/rotation_notes.md`）。
- **指の握り/伸ばし**: コントローラのトリガー(アナログ0..1)で開↔握り（`docs/hands_and_input.md`）。

### ⚠️ 本番パスの最重要事実（過去に地雷を踏んだ箇所）
**ライブ駆動の実体は `src/ops.py` の `MANEKKO_OT_start` modal → `_read_valid`**。
- ops は**自前で** `getDeviceToAbsoluteTrackingPose` を読み、`openvr_reader.world_to_blender`(z,x,y) で変換。
- **`TrackerReader` スレッド（`openvr_reader._loop`）は拡張では未使用**（開発MCP用）。reader と ops は
  座標変換が違う（reader=`svr_to_blender`(x,-z,y) / ops=`world_to_blender`(z,x,y)、90°差）。
- かつて `BONE_OFFSET_M`（ボーンオフセット）が reader にしか配線されず**本番で死んでいた**。v0.0.9 で
  `ops._read_valid` に `correct_to_bone` を配線して解消。**ops 側を直さないと本番に効かない**を肝に銘じる。

### パイプライン（位置＋回転）
1. `ops._read_valid`: 各 pose を読む → `correct_to_bone`（法線方向に骨へ寄せる, SteamVR系で適用）→
   位置=`world_to_blender`、姿勢=`world_rot_to_blender`(同じ軸写像 W@R) を `(pos, rot)` で返す。
2. `LiveDriver.calibrate`: Aポーズで位置オフセット＋**回転マウントオフセット** `M=R_raw_apose⁻¹·R_rest` を
   登録（`Calibration.rot_offset`、CONFIGにJSON保存・Start時自動ロード・旧JSONは位置のみで後方互換）。
   **回転は演者側(トラッカー局所)で扱う**＝Blender世界系の面倒な変換を避ける（位置キャリブが立ち位置を
   吸収するのと同じ構図。これがこの実装の勘所＝オーナー合意）。
3. `LiveDriver.step`: 位置=`raw+offset`、姿勢=`R_raw_live·M` を `solver.set_target_poses` へ。
4. `solver`: 各ロールに position FrameTask（hand=full weight）＋**全ロール orientation_cost=1e-1**。
   `orientation_roles` から1ロール外せばその部位だけ position-only に戻る（**切り分け用に設計**）。daqp QP。
5. `apply`: **FK関節角のみ**適用（ボール=quat→euler, ヒンジ=軸角, ルート=world pos+quat）。**無変更で回転対応**
   （hip=ルートは元々 quat、head等ボールは euler を適用済み）。非ルートに world行列/位置/scaleを焼くの禁止。

### トラッカー配置（物理・確定）
旧 elbow×2→**手首**(hand_l/r 駆動)、旧 knee_l→**胸**(chest=CC_Base_Spine02)、旧 knee_r=**休止/スペア**、
コントローラ×2→**手のひら(palm_l/r)**＝IK非対象（**指トリガー入力源**）。膝・肘は IK ドロップ。
`openvr_reader.SERIAL_TO_ROLE`/`BONE_OFFSET_M` 参照。

### コントローラ入力＆指（v0.1.1–0.1.5）— 詳細 `docs/hands_and_input.md`
- **レガシー `getControllerState` は `VRApplication_Overlay` で生きる**（Background では死。pose も維持）。
  `ops._init_vr` が Overlay で init（失敗時 Background フォールバック、`OPENVR_APP_TYPE` で切替）。
- トリガー(`rAxis[1].x`, 0..1)→ `ops._grip_from_ctrl` → `apply._apply_finger_curl` が指を rest↔fist 補間。
- 指の曲げ方向は実機調整して `apply.FINGER_CURL_DIR_DEG` に焼込み（L 親指200/他指270, R 親指160/他指90°,
  左右ミラー R=360−L）。指ボーンは `apply.finger_bone_names`＝Hand の全子孫（命名非依存）。録画で指もキー化。

### wheel bootstrap と sys.path ポリシー警告（重要・既知）
- 拡張は `__init__._ensure_wheels()` で同梱 `wheels/*.whl` を `_libs/` に展開し sys.path 先頭へ挿入。
  **この環境では必須**（マニフェスト `wheels=` だけだと `import openvr` が ModuleNotFoundError＝v0.1.3で確認）。
  Blender 自前の wheel 機構はここでは効かない。
- これで **「Policy violation with sys.path: ._libs」警告**が出るが**無害**（機能阻害なし）。
  アンインストールボタンが Add-ons UI に出ないのは**この警告とは無関係**（警告を消しても出なかった）。
  **アンインストール＝拡張フォルダ削除**（`.../extensions/user_default/manekko/`）。

### チューニングノブ（全部「マッピング＋定数」）
- `rig.PERFORMER_ARM_M=0.55 / PERFORMER_THIGH_M=0.47 / GLOBAL_HEIGHT_SCALE=1.80/1.59`（演者寸法）
- `apply.ROOT_HEIGHT_SCALE=1.0`（キャラ root 高さ配置）
- `solver.orientation_cost=1e-1`（弱め設定。追従弱→上げる0.3-0.5 / 暴れる→下げる）
- `solver.orientation_roles`（回転させる部位。捻れ・過剰拘束が出た部位を外す＝足が捻れたら足だけ外す等）
- `openvr_reader.BONE_OFFSET_M`（head8 hip6 chest6 wrist3 foot1 cm、トラッカー法線方向に骨へ寄せる量）
- `apply.FINGER_CURL_DIR_DEG`（指の握り方向・度。左右/親指別。`FINGER_CURL_ANGLE`=握り量）
- `ops.OPENVR_APP_TYPE`（"overlay"。レガシー入力が要らない/問題が出たら "background"）

### ビルド/リリース
`blender --command extension build --source-dir . --output-dir dist`（PowerShellは exe フルパス指定）。
版を上げる→ビルド→旧zip削除（検証済みは fallback で一時保持可）。**動作検証が通るたび commit/push**。

### 残作業 / 今後（休眠モード）
- **基本は完成。次の起動はバグ修正のとき**。バグが無ければ当分触らない。修正時はまず本セクション上の
  「本番パスの最重要事実」と該当 docs を読んでから着手（再調査の手間を省く）。
- **フェイス**: iPhone ARKit、**別プロジェクト**。本リポでは扱わない。
- 余力があれば: sys.path ポリシー違反を消す本筋（Blender の wheel 機構をこの環境で機能させる）。今は
  bootstrap で回避＝警告許容。トラックパッド(`rAxis[0]`)は生きているので将来の追加入力源に使える。

---

## START HERE（旧·P1着手＋トラッカー再配置計画時）— 2026-05-31 午前

**いまの状態**: `dist/manekko-0.0.6.zip` をビルド済みだが **未実機テスト**（オーナーは水泳へ、~6h後に戻って
Blender+SteamVR 起動→0.0.6 をインストール→Start でライブ確認、から再開予定）。0.0.6 には**未コミットの
P1**が入っている（コミット済みは bone-offset と wheel-bootstrap と肘ドロップまで）。

**今日やったこと（2026-05-31 午前）**:
1. **トラッカーのボーンオフセット補正**（commit `79ae66a`）: 各トラッカーの報告 pose を、その**ローカル法線方向
   に体内へ寄せて**から `svr_to_blender`。VIVE Tracker 3.0 の法線軸は実機計測で **-Z**（床に平置きで法線が天井
   ＝-Z）、ボーンは床側なので寄せる向きは **+Z**。`openvr_reader.correct_to_bone()`＋`BONE_OFFSET_M`
   {head 8cm, hip/elbow/knee 3cm, foot 1cm, 手=0}。回転行列があるので表裏の不定性は無い。実機で量＝設定値・
   向き＝床方向を検証。頭・足・膝が劇的に改善。
2. **wheel 自己ブートストラップ**（commit `c1174bf`）: Blender の拡張 wheel 自動展開が**この環境では動いて
   いなかった**（`extensions/.local` が一度も作られず `import openvr` 失敗。従来「動いた」のは開発用 temp-libs
   が sys.path に居たから）。`__init__._ensure_wheels()` が同梱 `wheels/*.whl` を `_libs/` に一度展開して
   sys.path 先頭へ（既に import 可なら no-op）。
3. **肘トラッカーを IK から除外**（commit `c1174bf`）: 腕は過剰拘束（Upperarm ball3 + Forearm hinge1 =
   4DOF vs 肘3+手首3=6）。肘＋不正確なコントローラ手首ターゲットが取り合い肘を固める。→ 手首で腕を駆動し
   肘スイベルは Posture が解決。肘トラッカーは読むだけ（戻すのは `rig.TRACKER_TO_BONE` の2行コメント解除）。
4. **P1: 演者寸法モデル化（未コミット・0.0.6・未テスト）= アーキテクチャの転換**。`docs` というより
   思想は下記「設計原理」を見よ。`rig.build_mjcf` が各ボディの親相対オフセットを**演者の肢長**にスケール
   （腕 肩→手首 0.55m、腿 hip→膝 0.47m、その他 `GLOBAL_HEIGHT_SCALE=1.80/1.59`）、向きはキャラ A-pose を保持。
   よって **solve の qpos ＝演者の真の関節角**（solve にキャラ寸法が入らない）。`apply` は原理的に無修正、
   calibration は自動清浄化（演者rest に登録）。**hip 高さノブは Stage 1(`live.py`) から Stage 2
   (`apply.ROOT_HEIGHT_SCALE`, 既定1.0) へ移設**。

**設計原理（2026-05-31 にオーナーと合意・最重要）**: カメラ/マーカー方式が効くのは **(1) 演者の完全FKを
演者寸法骨格で再構成 → (2) 別工程でキャラへリターゲット** と二段階を分けるから。スパーストラッカーは両者を
1つの濁った solve に潰すと失敗する。よって **mink は「演者のFKを作る道具」、リターゲットは別**。
seam をコード不変条件に: **Stage 1 の solve にキャラ情報を入れない**（演者寸法・演者rest・トラッカーのみ）。
キャラが出るのは全部 Stage 2（`apply` の角度リターゲット＋root高さ配置）。「完全」は無理でも「クリーン
（二重変換を作らない）」は取る、というインディ方針。**角度は最後に・段階的に**（部位別 orientation_cost を
0→正、head→pelvis→chest→feet→四肢 の順。剛体単一から、捻れる四肢は最後）。

**次回の再開手順**: ①Blender5.1+SteamVR起動 ②`dist/manekko-0.0.6.zip` インストール（初回有効化で `_libs/`
へ wheel 展開、数秒）③Start でP1ライブ確認＝キャラが歪まない/足がほぼ接地（浮くなら `apply.ROOT_HEIGHT_SCALE`
を下げる）/腕が手首から追従。**P1がおかしければ** `rig` のスケール比を 1.0 にすればキャラモデルに戻る。
身長合わせの保険として **CCキャラを180cmに伸ばす**手も可（オーナー曰く簡単）。
**チューニングノブ**: `rig.PERFORMER_ARM_M/PERFORMER_THIGH_M/GLOBAL_HEIGHT_SCALE`、`apply.ROOT_HEIGHT_SCALE`。

**次フェーズ＝トラッカー再配置（物理加工待ち・総合案その1）**: 肘トラッカー→**手首**（hand_l/r=L/R_Hand,
位置。コントローラ手首問題を解消）、片膝→**胸**（新role `chest`=`CC_Base_Spine02`, 位置, ロングバンド要購入）、
もう片膝→**スペア**、コントローラ→**手のひら**（トリガー/録画専用、将来 Hand の向き源）、**両膝は IK から
ドロップ**（足で駆動）。点で確認後に全部回転を送る。鎖骨は加工難度で今回見送り。コード変更は
`SERIAL_TO_ROLE`/`TRACKER_TO_BONE`/`ROLES`/`BONE_OFFSET_M`（手首・胸）のマッピング＋定数のみ（角度ゼロ）。
**リスク**: 両膝ドロップは脚が荷重・深屈伸するので Posture が膝の向きを誤推定しうる（スペアを膝に戻せるように）。

---

## START HERE（旧）— 2026-05-29 夕方

**何を作っているか**: SteamVR の VIVE Tracker 3.0（最大10点）をリアルタイムに取り込み、
Blender 上の Character Creator（CC）キャラを全身IKで動かす Blender 5.1 拡張。
ライブ駆動 + Action へのキー提録。IK は **mink**（MuJoCoベース差分IK / daqp QP）。

**リポ**: https://github.com/ysk424/manekko （private）/ branch `main`
**締切**: 月曜。

**🎉 2026-05-30: 拡張として完成・実機で「全部動く」**。トラッカー位置→mink IK→FK角→CCキャラを
リアルタイム駆動。N-panel "Manekko"（`src/ops.py`+`ui.py`）に **Start/Stop・Record・Calibrate** の
3ボタン。Record=押下→ビープ→5秒→ビープで**アクティブActionへキー提録**しながらフレーム送り。
Calibrate=押下→5秒→Aポーズ取得を**ユーザCONFIGにJSON保存**（`src/calibration_io.py`、Start時に自動読込）。
ヌル空間ドリフトは `posture_cost` 1e-2→**1e-1** で解消（`solver.py`既定値）。トリガー検出は断念
（XInput/winmm/IVRInput いずれも背景アプリで不可）→録画はビープ＋ボタン方式。
ビルド: `blender --command extension build --source-dir . --output-dir dist`（manifest は
`paths_exclude_pattern` 方式。`paths=["src"]` だと wheels 抜けの2KB zip になる）。
**次回の残作業：IK チューニング＋細かいキャリブ補正**（登録はキャラ/姿勢依存・前方合わせは固定+90°）。
公開予定（参照実装として public 化、README は英語・Claude Code 製を明記）。
詳細 `docs/live_driving_notes.md` / `docs/mink_pitfalls.md`。`src/live_ops.py` は `src/ops.py` に統合・削除。

### いま何ができているか（全部実機検証済み）

IKコアのパイプライン **トラッカー位置 → mink solve → CCボーン** が合成データで end-to-end 動作確認済み:

1. `src/rig.py` — CC_Base アーマチュアから縮約 MuJoCo MJCF を生成。
   A-pose を **誤差1e-6m** で再現。21ボディ（free root + ball×13 + hinge×4 + weld×3）、nq=63/nv=49。
   **各ボディフレームを対応ボーンのローカル rest フレームに整列**（B案・2026-05-30）。これにより関節 qpos が
   そのまま「ボーン局所の rest からの変位」になり、apply は quat→Euler の自明変換で済む。
   body⇔bone対応・tracker役割⇔body対応・rest world行列を返す。
2. `src/solver.py` — mink Configuration + 10点 **position-only** FrameTask + A-pose PostureTask、daqp。
   安定（rest残差0）、到達可能ポーズを ~7mm で復元、**1.7ms/4反復**。
3. `src/apply.py` — **角度のみリターゲット**（2026-05-30 にB案で全面書き換え）。ボーンに渡すのは
   **rest(A-pose)からのFK関節角だけ**（ボール=quat→Euler、ヒンジ=軸角→Euler、weldはbasis単位行列にreset）。
   **ルートのみ例外でグローバル位置＋角度**（スケールは明示除去）。ワールド行列/位置/スケールを非ルートボーンに
   焼くのは禁止（演者とキャラの体格差のため位置は転写不可、角度だけが体格非依存で移る）。
   検証済（2026-05-30）: 角度のみで全身ポーズを **8μm** で再構成、全ボーン scale=1.0、rest再現8μm。
   per-bone update を廃し最後に1回 view_layer.update（旧版の25ms問題も解消）。
   ※ 旧版（ワールド行列をボーンに焼く方式）はルートに100倍スケールが漏れるバグがあった。旧「5μm検証」は
   位置のみのチェックでスケール汚染を見逃していた。
4. `src/openvr_reader.py` — openvr を別スレッドでポーリング（bpy 非タッチ）。`TrackerReader`(start/stop/
   snapshot/device_table/assign)、座標変換 `svr_to_blender`、`Calibration`(A-pose位置オフセット)。
   **実機検証済**（2026-05-30）: HMD+ライトハウス4台を valid pose で列挙、軸入替 `(x,-z,y)` を天井
   ライトハウス(高さ≈2.1m→Blender Z≈2.1m)で裏取り、assign→snapshot フルパス・stop クリーン。

→ 当初の技術リスク（mink成立性・依存・OpenVR接続・座標/単位・モデル生成・可解性・Blender反映）は**全消化**。
残りは既知技術の組み立て。

### 次にやること（優先順）

**task #2 の残り** — リーダー/座標変換/Calibration は**完了・実機検証済**。
残るは A-pose キャリブレーションの結線（`Calibration.from_apose` に rig の rest body 位置を渡す）。
これは rest 位置取得が要るので task#3 のライブループに統合する。

**task #3（本番統合）** — ★実機検証が進行中。詳細は **`docs/live_driving_notes.md`**（必読）。
- **ライブは modal operator 必須**（`wm.event_timer`）。ブロッキングループは画面に提示されない（検証済）。
  hip→ルートの3人称追従を modal で**実機確認済み**（プロトタイプは notes に収録）。
- 残：プロトタイプを本番化＝**全身IK**（10点 position 目標を mink へ）＋ `apply` で関節角適用、
  ルートは `world_to_blender` 位置で駆動、beep2 で `Calibration` 結線。
- **録画は beep 方式**（トリガー不可のため。後述）：start→beep1→5秒(Aポーズ)→beep2(キャリブ＆録画開始)、
  停止はエクステンションのボタン。Action へキー提録。
- N-panel UI（Start/Stop/Calibrate/Record）。`apply` 速度は24fpsで急がない。

### 開発ループ（重要・これで反復している）

ライブ Blender 5.1 に MCP 接続し、`src/` を sys.path 経由で読み込んで試す。

```python
import sys, importlib.util
libs = r"C:\Users\azoo\AppData\Local\Temp\manekko_libs"   # ← 後述。消えてたら再作成
if libs not in sys.path: sys.path.append(libs)
def load(name, path):
    s = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(s)
    sys.modules[name] = m          # ★ dataclass が自分のモジュールを参照するので必須
    s.loader.exec_module(m); return m
rig = load("manekko_rig",   r"C:\Users\azoo\git\manekko\src\rig.py")
solv= load("manekko_solver",r"C:\Users\azoo\git\manekko\src\solver.py")
ap  = load("manekko_apply", r"C:\Users\azoo\git\manekko\src\apply.py")
import bpy, mujoco
arm = bpy.data.objects["Toon Neutral_F"]   # ← CCキャラのアーマチュア名
rm  = rig.build_mjcf(arm)
model = mujoco.MjModel.from_xml_string(rm.mjcf)
```

**`manekko_libs` 再作成**（Temp なので再起動で消える可能性。wheels/ はローカルに残っている）:
```
& "C:\Program Files\Blender Foundation\Blender 5.1\5.1\python\bin\python.exe" -m pip install `
  --no-index --find-links "C:\Users\azoo\git\manekko\wheels" --target "C:\Users\azoo\AppData\Local\Temp\manekko_libs" mink openvr
# 念のため: rm -f wheels内 numpy はそもそも入れていない / install後に target の numpy* を削除してもよい
```
（最終的には拡張zipに同梱した wheels が Blender により自動で sys.path に載るので、この手順は開発専用。）

### 検証済みの確定事実

| 項目 | 値 |
|---|---|
| Blender / Python | 5.1 / 3.13.9 win-amd64 |
| 依存 wheel (cp313 win_amd64) | mink 1.1.1, mujoco 3.9.0, openvr 2.12.1401, qpsolvers 4.12.0 + daqp 0.8.7, scipy 1.17.1 ほか。`wheels/` に13個同梱（numpy除外） |
| numpy | Blender 同梱 2.3.4（numpy2 ABI）を流用。**同梱しない** |
| CCアーマチュア | オブジェクト名 `Toon Neutral_F`、標準 CC4 `CC_Base` 101ボーン |
| 単位/座標 | armature scale=0.01、scene METRIC unit_scale=1 → `arm.matrix_world @ head_local` がワールド**メートル**。身長~1.59m。MJCFはワールドmで構築 |
| OpenVR | `openvr.init(VRApplication_Background)` 成功。HMD=`LHR-CD5AF0FE`(VIVE Pro 2)。ライトハウス4台は class **5(DisplayRedirect)** を返す（VIVE基地局の癖／対象外）。トラッカー(class 3 GenericTracker)は点灯＋装着で出る。`getDeviceToAbsoluteTrackingPose(Standing)` 使用 |

### 10点トラッカー割当（IKタスク目標 / position-only）

`hip→Hip, head→Head, hand_l/r→L/R_Hand, foot_l/r→L/R_Foot, elbow_l/r→L/R_Forearm, knee_l/r→L/R_Calf`
（`rig.py: TRACKER_TO_BONE`）

### 決定事項（背景つき）

- **FrameTask は位置のみ**（orientation_cost=0）。理由: 角度/回転情報は地雷にハマりやすい（ユーザー方針）。
  姿勢は PostureTask の A-pose 寄せで決める。後で 6DOF に拡張可能。
- **キャラのボーンに渡すのはFK関節角だけ（ルートのみグローバル位置＋角度を例外で渡す）**（ユーザー方針・B案）。
  理由: モーキャップ演者とキャラは体格が違うので**ワールド位置は転写できない**。関節角（rest からの変位）だけが
  体格非依存で正しく移る。位置情報の役割は hip トラッカー→ルートのみ。ワールド行列/スケールをボーンに焼くの禁止。
  実装: `rig.py` がボディをボーン局所フレームに整列 → `apply.py` は qpos を Euler 変位として渡すだけ。
- **24fps で十分**（動画は Cycles が遅く 24fps 制作）。apply の25msは許容。速度は完成後に
  一部を外部 C++ に出す案あり（後回し）。
- **ライブ表示は modal operator 必須**（ブロッキングPythonループは画面に提示されない＝実機検証で確定）。
- **手はコントローラ、トリガーはレガシーAPIで取得不可**（IVRInput必要・後回し）。よって**録画はトリガーでなく
  beep方式**：start→beep1→5秒(Aポーズ)→beep2(キャリブ＆録画開始)、停止はエクステンションのボタン。
- **正面合わせ＝固定+90°ヨー**。部屋は画面が world -X、演者は画面を向く＝正面 world -X をキャラ正面 Blender -Y に
  合わせる。`svr_to_blender`(=(x,-z,y)) の後に +90°、合成で **SteamVR(x,y,z)→Blender(z,x,y)** = `world_to_blender`。
  実機検証済（前=-Y, 左=+X）。**鏡映ではない**（3点歩行テストで vecX→vecY=+87°、ヨー単一で確定）。部屋配置が
  変われば hip 向きからヨーを算出。
- **commit/push は節目ごとに自動**（動作検証が通るたび push）。
- **ハード運用**: SteamVR はスタンバイ↔プレイの違いのみ重要。トラッカーはライトハウス4台点灯＋装着で出る。
  HMDは基準で常時監視（トラッキングには非使用）。ユーザーがGUI/管理者PWSH/起動停止を操作。
  **この Code は停止時にビープ**するので、ハード操作を頼んで止まればユーザーが気づく。

### 既知の宿題 / 地雷

- **A-pose は特異姿勢**（肘膝ほぼ伸展）。差分IKが特異点で暴れる/裏返るリスク。`docs/mink_pitfalls.md` 参照。
- **関節限界が未設定**（過伸展・裏返り）。MJCF に range か mink.ConfigurationLimit を検討。
- `apply` 25ms（view_layer.update×21）。最適化は task #3 で。
- mink solve は `NoSolutionFound` を投げうる → ライブ loop は try/except で前ポーズ保持。

詳しい mink/MuJoCo の地雷は **`docs/mink_pitfalls.md`**（収集済み）。

### ファイル地図

```
blender_manifest.toml  — Blender5.1拡張, wheels同梱リスト(numpy除外), id=manekko
__init__.py            — register/unregister, manekko_mode enum (IDLE/LIVE/RECORD) ※まだ足場
ui.py                  — N-panel "Manekko" ※まだ足場
src/rig.py             — MJCF生成（検証済, ボーン局所フレーム整列）
src/solver.py          — mink ソルバ（検証済）
src/apply.py           — ボーン適用＝FK角のみ（検証済, B案）
src/openvr_reader.py   — トラッカー取得/座標変換/Calibration。SERIAL_TO_ROLE確定, world_to_blender(+90°)
src/live.py            — LiveDriver（snapshot→calibrate→solve→apply オーケストレーション）
scripts/fetch_wheels.ps1 — wheel取得
docs/mink_pitfalls.md  — 地雷集
docs/live_driving_notes.md — ★ライブ/modal/座標/トリガーの実機検証メモ＋検証済みプロトタイプ（task#3前に必読）
wheels/                — 同梱wheel（.gitignore, ローカルのみ）
```
