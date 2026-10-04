# 画質の段階 (-quality) と HDRP 版

## 画質の段階 `-quality low | medium | high | auto`

起動引数 (launch・`scripts/shots.sh` は `QUALITY=`)。既定 `auto` は GPU を見て選ぶ (`RenderQuality.cs`)。選んだ段階と理由は
起動ログ `[RenderQuality]` と `bench.md` に出る。

| 段階 | 中身 (Built-in) | 中身 (HDRP 版) | auto で選ばれる GPU |
|---|---|---|---|
| low | 今までと同じ描画。森の影だけ切る | 後処理なし・影 150 m | ソフトウェア描画・内蔵 GPU (Intel・AMD APU)・ビデオメモリ 2 GB 未満。**`-mlagents` のときは常に low** |
| medium | 影 300 m・画面の AA 2x | AO・ブルーム・ACES・影 300 m | ビデオメモリ 2〜6 GB (例: Radeon RX 5300M 3 GB = 評価用の MSI Bravo 15) |
| high | 影 600 m・AA 4x・LOD 2 倍 | 影 600 m・立体の霧・立体の雲 | ビデオメモリ 6 GB 以上 |

配信するセンサ画像 (車載カメラ) は段階によらず同じ (AA なし・後処理は実カメラ風の SensorPost だけ)。

## HDRP 版のプロジェクト

元の `unity/MinicarSim` (Built-in) はそのまま残し、**HDRP 版は別のプロジェクト `unity/MinicarSimHDRP` をスクリプトで作る** (リポジトリには入れない)。
スクリプト (C#) は同じものを使い、HDRP 版にだけスクリプト定義 `MINICAR_HDRP` を付けて `RenderCompat.Hdrp.cs` を有効にする。
コースと車は今まで通り Standard の材質で組み、組み上がったあとで HDRP の材質・光・空・霞・露出に置き換える。

```bash
./scripts/migrate_hdrp.sh             # 同期 → ① HDRP パッケージ → ② 設定 → ③ ビルド → ~/jetracer/unity/player_hdrp に配置
./scripts/migrate_hdrp.sh --shots     # さらに Built-in 版と HDRP 版を同じ位置で撮って並べる → shots/compare_builtin_hdrp_<日時>.png
STEPS="2 3 install" ./scripts/migrate_hdrp.sh   # 途中の段だけ。ただし sync をしたら 1・2 もやり直す (ProjectSettings が元に戻るため)
```

| 段 | すること | ログ |
|---|---|---|
| sync | `Assets/{Minicar,Scenes,StreamingAssets}`・`ProjectSettings`・`Packages/manifest.json` を写す。HDRP が作ったものと Library は残す | — |
| 1 | `com.unity.render-pipelines.high-definition` を入れる (この Unity に合う版)。`MINICAR_HDRP` を足す (`HdrpMigration.Phase1`) | `unity/MinicarSimHDRP/Logs/hdrp_phase1.log` |
| 2 | HDRP の設定アセット `Assets/MinicarHDRP/MinicarHDRP.asset` を全画質に割り当て、色空間 Linear・Linux は Vulkan・Global Settings・`Mat_HDLit` と材質の変形 (`Resources/HDVariants`: 切り抜き・半透明・法線・detail。プレイヤーでシェーダが削られないように) (`HdrpSetup.Phase2`) | `hdrp_phase2.log` |
| 3 | course.json を入れてプレイヤーをビルド (`MinicarBuild.SetupAndBuild`) | `hdrp_build.log` |
| install | `setup_unity_player.sh` で配置 | — |

HDRP 版を launch で使うときは `unity_player:=~/jetracer/unity/player_hdrp/MinicarSim.x86_64`。

### 置き換えの中身 (`RenderCompat.Hdrp.cs`)

- 材質: Standard → HDRP/Lit (色・テクスチャ・タイル・法線・滑らかさ・金属・切り抜き・半透明・detail)。Unlit・Sprites (矢印板・文字・雲のドーム) はそのまま
- 富士: 太陽 100,000 lux・物理的な空・露出 EV100 14.4 (Built-in の明るさに合わせた値)・霞は今の指数の霞と同じ減衰で、高さ 2.5 km で薄れる
- ミニカーの会場: 光の強さは今の値のまま、露出 EV100 −1.9 で Built-in と同じ明るさに換算。環境光は上・横・下の 3 色の空
- センサカメラには後処理 (トーンマップ・ブルーム) と AO を掛けない。HDRP 既定の Volume が入れる AO・ブルーム・モーションブラーは low で 0、モーションブラーは常に 0
- エピソードごとに作られる床テープ・観戦者も `RenderCompat.Refresh` で置き換える

### 注意

- **学習 (ML-Agents)・ミニカーの会場の検出器の評価は Built-in 版で行う。** HDRP 版は色空間 (Linear)・光の計算が違うので、センサ画像の見え方が変わる。HDRP 版は表示・動画用
- Built-in 版は Gamma 色空間、HDRP 版は Linear なので、同じ光の強さでも中間の明るさが少し明るく出る (例: 0.55 → 約 0.76)。露出 (`kCircuitEV`・`kLegacyEV`) で合わせる
- HDRP は Linux では Vulkan が要る。AMD は Mesa (RADV)、NVIDIA は独自ドライバで動く。内蔵 GPU やビデオメモリ 4 GB 未満では重い (Built-in 版の medium を使う)
- HDRP の API は Unity 6 (HDRP 17) を前提に書いてあり、まだ実機でコンパイルしていない。③ でエラーが出たら `RenderCompat.Hdrp.cs` か `Editor/HdrpSetup.cs` を直す。
  Global Settings の警告が出たら、エディタで `unity/MinicarSimHDRP` を開き Window > Rendering > HDRP Wizard の Fix All を 1 回押して、`STEPS="3 install"` で続ける
- 元に戻す: `unity/MinicarSimHDRP` と `~/jetracer/unity/player_hdrp` を消すだけ (元のプロジェクトは変わっていない)
