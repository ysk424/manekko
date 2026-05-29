# Manekko — Working Notes (read this first)

Handoff log for future Claude Code sessions. Update when phases close.
Communication is Japanese. Owner: `azoo` / `ysk424` (ysk424@hotmail.com).

---

## ⚠️ START HERE — 2026-05-29 夕方

**何を作っているか**: SteamVR の VIVE Tracker 3.0（最大10点）をリアルタイムに取り込み、
Blender 上の Character Creator（CC）キャラを全身IKで動かす Blender 5.1 拡張。
ライブ駆動 + Action へのキー提録。IK は **mink**（MuJoCoベース差分IK / daqp QP）。

**リポ**: https://github.com/ysk424/manekko （private）/ branch `main` / 最終 push `c9759a6`
**締切**: 月曜。明朝（5/30 早朝）から再開。

### いま何ができているか（全部実機検証済み）

IKコアのパイプライン **トラッカー位置 → mink solve → CCボーン** が合成データで end-to-end 動作確認済み:

1. `src/rig.py` — CC_Base アーマチュアから縮約 MuJoCo MJCF を生成。
   A-pose を **誤差1e-6m** で再現。21ボディ（free root + ball×13 + hinge×4 + weld×3）、nq=63/nv=49。
   body⇔bone対応・tracker役割⇔body対応・rest world行列を返す。
2. `src/solver.py` — mink Configuration + 10点 **position-only** FrameTask + A-pose PostureTask、daqp。
   安定（rest残差0）、到達可能ポーズを ~7mm で復元、**1.7ms/4反復**。
3. `src/apply.py` — solve結果の MuJoCo body world変換を CC_Base pose bone に適用（誤差**5μm**）。
   絶対matrix・root-first・ボーン毎 view_layer.update。現状 **~25ms**（24fps運用なので許容、後で最適化）。
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

**task #3（本番統合）**
- ライブ駆動 modal operator（`wm.event_timer`, main thread で snapshot→solve→apply）。
- `apply` の25ms最適化（matrix_basis直接計算 or update回数削減）。**24fpsでよいので急がない**。
- Action へキー提録（録画ボタン）。N-panel UI（Start/Stop/Calibrate/Record）。

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
- **24fps で十分**（動画は Cycles が遅く 24fps 制作）。apply の25msは許容。速度は完成後に
  一部を外部 C++ に出す案あり（後回し）。
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
src/rig.py             — MJCF生成（検証済）
src/solver.py          — mink ソルバ（検証済）
src/apply.py           — ボーン適用（検証済）
scripts/fetch_wheels.ps1 — wheel取得
docs/mink_pitfalls.md  — 地雷集
wheels/                — 同梱wheel（.gitignore, ローカルのみ）
```
