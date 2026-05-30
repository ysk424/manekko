# Manekko — Working Notes (read this first)

Handoff log for future Claude Code sessions. Update when phases close.
Communication is Japanese. Owner: `azoo` / `ysk424` (ysk424@hotmail.com).

---

## ⚠️ START HERE — 2026-05-29 夕方

**何を作っているか**: SteamVR の VIVE Tracker 3.0（最大10点）をリアルタイムに取り込み、
Blender 上の Character Creator（CC）キャラを全身IKで動かす Blender 5.1 拡張。
ライブ駆動 + Action へのキー提録。IK は **mink**（MuJoCoベース差分IK / daqp QP）。

**リポ**: https://github.com/ysk424/manekko （private）/ branch `main`
**締切**: 月曜。

**🎉 2026-05-30: 全身ライブモーキャップ成立**（実機）。トラッカー位置→mink IK→FK角→CCキャラを
リアルタイム駆動できている（`src/live_ops.py`）。途中で出た**ヌル空間ドリフト**（足踏みでねじれ累積）は
`posture_cost` を 1e-2→**1e-1** に上げて解消（`solver.py` 既定値）。残：キャリブ微調整（後回し）、
録画・UI、拡張への結線。詳細 `docs/live_driving_notes.md` / `docs/mink_pitfalls.md`。

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
