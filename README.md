# Manekko

SteamVR の VIVE Tracker 3.0（最大10点）をリアルタイムに取り込み、Blender 上の
Character Creator（CC）キャラクターを全身IKで動かす Blender エクステンション。
ライブ駆動 + Action へのキーフレーム提録に対応する本番モーキャプツール。

- **IK**: [mink](https://github.com/kevinzakka/mink)（MuJoCo ベースの全身タスクIK / QP）
- **トラッカー取得**: OpenVR を直接読む（`openvr` = pyopenvr）
- **キャリブレーション**: A ポーズ（CC 素体ポーズが A ポーズ）
- **対象**: Windows x64 / Blender 5.1 / Python 3.13

## 検証済みの確定事項（2026-05-29）

| 項目 | 値 |
|---|---|
| Blender / Python | 5.1 / 3.13.9 win-amd64 |
| 依存 wheel（cp313 win_amd64） | mink 1.1.1, mujoco 3.9.0, openvr 2.12.1401, qpsolvers 4.12.0 + daqp 0.8.7, scipy 1.17.1 ほか — **全て取得可** |
| numpy | Blender 同梱 2.3.4 を流用（numpy2 ABI 互換のため**同梱しない**） |
| OpenVR → SteamVR 接続 | `openvr.init(VRApplication_Background)` + デバイス列挙 成功 |
| 対象リグ | 標準 CC4 `CC_Base` スケルトン（101 ボーン）。腕は水平から約 -29°＝**A-pose 確定**。単位は cm スケール、ワールド原点 |

## アーキテクチャ

```
VIVE Tracker (OpenVR, ~90Hz)
  → 別スレッドでポーズ行列を取得し最新スナップショットを保持（bpy には触れない）
  → modal operator の wm.event_timer が main thread で:
        スナップショット取得
        → SteamVR座標(Y-up/m) → Blender(Z-up/cm) 変換
        → mink の FrameTask 目標を更新（10点）
        → daqp で QP solve（MuJoCo モデル上）
        → 関節角を CC_Base ボーンへ適用
  → （録画中なら）各ボーンの rotation/location を Action にキーベイク
```

mink は **MuJoCo モデル**上で解く。CC_Base のレスト姿勢から生成した humanoid MJCF と
CC ボーンの対応表を保持し、両者のレスト姿勢（A ポーズ）を一致させる。

### 10点トラッカー割当（IKタスク目標）

頭・腰・左右手・左右足（6）＋ 左右肘・左右膝（4）＝ 10点。
肘膝トラッカーで関節の向きを安定させる。

## 開発ループ

ライブの Blender 5.1 へ MCP 接続し、`src/` の python を `sys.path` 経由で読み込んで反復。
最終的に `scripts/fetch_wheels.ps1` で wheel を `wheels/` へ取得し、エクステンション zip に同梱する。

## 状態

初期足場。タスクは [基盤構築 / IKコア / 本番統合] の3分類で進行。
