# 画質の段階 (-quality) と URP 版・HDRP 版

## 画質の段階 `-quality low | medium | high | auto`

起動引数 (launch・`scripts/shots.sh` は `QUALITY=`)。既定 `auto` は GPU を見て選ぶ (`RenderQuality.cs`)。選んだ段階と理由は
起動ログ `[RenderQuality]` と `bench.md` に出る。

| 段階 | 中身 (Built-in) | 中身 (HDRP 版) | auto で選ばれる GPU |
|---|---|---|---|
| low | 今までと同じ描画。森の影だけ切る | 後処理なし・影 150 m | ソフトウェア描画・内蔵 GPU (Intel・AMD APU)・ビデオメモリ 2 GB 未満。**`-mlagents` のときは常に low** |
| medium | 影 300 m・画面の AA 2x | AO・ブルーム・ACES・影 300 m | ビデオメモリ 2〜6 GB (例: Radeon RX 5300M 3 GB = 評価用の MSI Bravo 15) |
| high | 影 600 m・AA 4x・LOD 2 倍 | 影 600 m・立体の霧・立体の雲 | ビデオメモリ 6 GB 以上 |

配信するセンサ画像 (車載カメラ) は段階によらず同じ (AA なし・後処理は実カメラ風の SensorPost だけ)。

medium 以上で実車スケールのコースには、次も足す (Built-in 版):

- 画面表示の後処理 `ViewPost` (明るい所のにじみ・コントラスト・周辺減光)。表示用のカメラだけで、車載カメラには掛けない
- 周りの景色の映り込み (路面の上に置いたプローブを起動時に 1 回だけ写す)。車の塗装とガラスに空と地平線が映る

ノート PC (内蔵 + 単体 GPU) では、launch と `scripts/shots.sh` が `DRI_PRIME=1` を既定にして単体 GPU を使う。
基準の数値は [render_baseline.md](render_baseline.md)。

## 3 つの版と自動の切り替え (Built-in / URP / HDRP)

描画の仕組みが違う 3 つのプレイヤーがある。**見せる用の起動は、GPU を見て自動で選ぶ** (`scripts/pick_unity_player.sh`)。

| 版 | プレイヤー | 向き | 選ばれる GPU (ビデオメモリ) |
|---|---|---|---|
| Built-in | `~/jetracer/unity/player` | 軽い。**学習・検出器の評価・画像で走る方策はこの版** | 2 GB 未満・Intel の内蔵だけ・不明 |
| URP | `~/jetracer/unity/player_urp` | 中間。部屋の会場・後処理つきで軽い | 2 GB 以上 6 GB 未満 (例: Radeon RX 5300M 3 GB) |
| HDRP | `~/jetracer/unity/player_hdrp` | 重い・きれい (映り込み・際の陰り・物理的な空) | 6 GB 以上 |

- 境目は `-quality auto` (low / medium / high) と同じ。選んだ版のプレイヤーが無ければ 1 つ軽い版へ下げる。選んだ理由は標準エラーに出る
- 指定するとき: `JETRACER_PIPELINE=builtin|urp|hdrp` (または `PLAYER=<実行ファイル>`)
- 自動が既定なのは `tools/race/race3.sh`・`tools/race/race3_fuji.sh` だけ。`race3.sh` で方策を画像で走らせるとき (`BLUE_MODEL`) は Built-in に固定
- launch は `unity_player:=auto` と書いたときだけ自動 (`mlagents:=true` のときは Built-in)。リポジトリが `~/jetracer/jetracer_sim` 以外にあるときは `JETRACER_SIM_ROOT` を渡す
- ★URP 版・HDRP 版は配信するセンサ画像の見え方が Built-in と違う。記録 (`record.sh`)・学習・検出器の評価には使わない

評価用 PC (RX 5300M) での速さ (富士・`-quality medium`・3 分割の画面・1920×1080・計測 400 フレーム、2026-10-05):
Built-in 142 fps、URP 201 fps、HDRP 40 fps。ミニカーの部屋の会場の 3 台戦は URP (medium)・HDRP (high) とも 60 fps (上限)。

## URP 版のプロジェクト

HDRP 版と同じ方式で、**別のプロジェクト `unity/MinicarSimURP` をスクリプトで作る** (リポジトリには入れない)。スクリプト定義 `MINICAR_URP` を付けて `RenderCompat.Urp.cs` を有効にする。

```bash
./scripts/migrate_urp.sh              # 同期 → ① URP パッケージ → ② 設定 → ③ ビルド → ~/jetracer/unity/player_urp に配置
STEPS="3 install" ./scripts/migrate_urp.sh   # ソースを直したあと (sync をやり直したら 1・2 も要る)
```

`migrate_urp.sh` と `migrate_hdrp.sh` の中身は共通の `scripts/migrate_pipeline.sh <urp|hdrp>`。

- 設定 (`Editor/UrpSetup.cs`): 設定アセット `Assets/MinicarURP/MinicarURP.asset` と描画器、色空間 Linear、HDR・深度テクスチャ・影 4096、`Mat_URPLit` と材質の変形 (`Resources/URPVariants`)。描画 API は Built-in 版と同じ (この PC では OpenGLCore)
- 材質: Standard → Universal Render Pipeline/Lit。車のクリア層 (2 枚目の材質) は外し、塗装のつやを上げる
- 光: Linear でも Built-in (Gamma) と同じ明るさに見えるよう、光の強さを 2.2 乗して入れる
- 影・アンチエイリアス: コードが QualitySettings に入れた値を設定アセットへ写す
- 後処理 (medium 以上・表示用のカメラだけ): トーンマップ (Neutral)・ブルーム。部屋の会場は遠くのぼけ・周辺減光・コントラスト。映り込みと際の陰り (SSR・AO) は入れていない
- ミニカーの会場は URP 版でも部屋 (`-venue room`) が既定
- 分かっている違い: 地面の detail は 1 番目の UV に乗る (Built-in は 2 番目)。Built-in 用の画面の後処理 `ViewPost` は URP では働かない (上の後処理が代わり)

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

### ミニカーの会場を実際の部屋らしく (`-venue room`)

HDRP 版では、ミニカーの会場を実際にコースを組んだ部屋に寄せて描く (`CourseBuilder.Room.cs`。既定: HDRP 版 = `room`、Built-in 版 = `plain`)。
参考にしたのは会場の動画と「△3 コース・レギュレーション」の使用部材。コースの形・寸法・色の区別は `plain` と同じ。

| もの | 中身 |
|---|---|
| 部屋 | クリーム色の壁、+y 側にアルミサッシの 3 連窓 (外は白く飛んだ屋外)、蛍光灯の並ぶ天井 (高さ 2.7 m)、ベージュの床の上に広くパンチカーペット |
| 備品 | 窓際のえんじ色の長椅子、濃紺のついたて、巻いたカーペット、重ねた白い椅子、机とノート PC、時計・掲示・額 |
| カーペット | リックパンチカーペットの濃いグレー。濃淡の繊維が混じったフェルト (法線つき) |
| 高 μ 路 | 人工芝 (芝丈 30 mm・フレッシュグリーン)、縁は緑のテープ |
| 低 μ 路 | 白い PTFE シート (つやあり)、縁は緑のテープ |
| でこぼこ道 | 水色の風呂すべり止めマット (ストーン柄・40×80 cm) 4 枚、縁と継ぎ目は青のテープ |
| 壁板 | 塗装した木の板 (つやあり)。足は鉄板 150×150×4 mm + M20 ボルト + 塩ビ管 |
| 駐車枠の番号 | 枠と同じ色のテープを文字の形に貼ったもの |
| ライトかく乱 | ミラーボール型のステージライト。色の点が床に散ってゆっくり回る (床に置いた薄い円の集まり)。色と明滅は従来どおり 7 色を 2 秒ごと |
| 車 | 屋根のセンサマスト (カメラの柱) を付けない |
| 光と後処理 | plain より明るい光 (天井 ×1.9・環境 ×2.1)。medium 以上で映り込み・際の陰り・遠くのぼけ・周辺減光 |

`-venue plain` で今までの会場に戻る。Built-in 版でも `-venue room` で部屋は出るが、後処理は付かない。
★配信するセンサ画像にも部屋が写るので、room は表示・動画用。学習・検出器の評価は Built-in 版の plain で行う。

### 注意

- **学習 (ML-Agents)・ミニカーの会場の検出器の評価は Built-in 版で行う。** HDRP 版は色空間 (Linear)・光の計算が違うので、センサ画像の見え方が変わる。HDRP 版は表示・動画用
- Built-in 版は Gamma 色空間、HDRP 版は Linear なので、同じ光の強さでも中間の明るさが少し明るく出る (例: 0.55 → 約 0.76)。露出 (`kCircuitEV`・`kLegacyEV`) で合わせる
- HDRP は Linux では Vulkan が要る。AMD は Mesa (RADV)、NVIDIA は独自ドライバで動く。内蔵 GPU やビデオメモリ 4 GB 未満では重い (Built-in 版の medium を使う)
- 評価用 PC (Unity 6000.0.83f1・HDRP 17・RX 5300M) で全段が通ることを確かめた (2026-10-05)。直したのは Global Settings の型の参照 1 か所と露出 (`kCircuitEV` 14.4 → 12.9)。速さは [render_baseline.md](render_baseline.md)。③ でエラーが出たら `RenderCompat.Hdrp.cs` か `Editor/HdrpSetup.cs` を直す。
  Global Settings の警告が出たら、エディタで `unity/MinicarSimHDRP` を開き Window > Rendering > HDRP Wizard の Fix All を 1 回押して、`STEPS="3 install"` で続ける
- 元に戻す: `unity/MinicarSimHDRP` と `~/jetracer/unity/player_hdrp` を消すだけ (元のプロジェクトは変わっていない)
