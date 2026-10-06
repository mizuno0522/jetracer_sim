# 3 つの版 (Built-in・URP・HDRP) と画質の段階 (-quality)

## 3 つの版は同じ仕様の絵を出す

描画の仕組みが違う 3 つのプレイヤーがある。**出す絵の仕様は同じで、違うのは PC への負荷だけ** (2026-10-05 水野)。
HDRP 版の絵が正で、URP 版・Built-in 版はそれに合わせてある。見せる用の起動は GPU を見て自動で選ぶ (`scripts/pick_unity_player.sh`)。

| 版 | プレイヤー | 負荷 | 選ばれる GPU (ビデオメモリ) |
|---|---|---|---|
| Built-in | `~/jetracer/unity/player` | 軽い。学習 (`-mlagents`) はこの版 | 2 GB 未満・Intel の内蔵だけ・不明 |
| URP | `~/jetracer/unity/player_urp` | 中間 | 2 GB 以上 6 GB 未満 (例: Radeon RX 5300M 3 GB) |
| HDRP | `~/jetracer/unity/player_hdrp` | 重い | 6 GB 以上 |

そろえてあるもの (どの版も同じ):

- 場面: 富士、車体、ミニカーのコースそのもの (走路・壁板・区域・矢印板・暗幕・ミラーボール)。以前の灰色の床の会場 (`-venue plain`) は無くした。
  ★例外: ミニカーの会場の周り (部屋の壁・窓・天井・備品) は URP 版・HDRP 版だけ。**Built-in 版は部屋を描かず、地面と空だけ**
  (`RenderCompat.SimpleVenue`。いちばん軽い版を軽くするため。2026-10-06 水野)
- 色空間は Linear。光の強さ・色・環境光・露出
- 富士の空の色と明るさ、霞の色、雲 (雲の絵のドーム)、建物の屋根の影
- 画面表示の後処理の種類と強さ (下の表)。センサカメラ (配信する画像) にはどの版も後処理を掛けない
- 画質の段階 (`-quality`) の中身

見た目の数値は `RenderCompat.cs` の 1 か所 (`GetLook`・`CircuitSun` など) に置き、3 つの版が同じ値を読む。版ごとのファイルは、その値を
それぞれの仕組みに渡すだけ: `RenderCompat.Builtin.cs` + `ViewPost` (自前のシェーダ)、`RenderCompat.Urp.cs`、`RenderCompat.Hdrp.cs`。

残っている違い (仕組みの違いで、同じにできていないもの):

- 床や板への映り込み (SSR) は HDRP 版だけ。URP 版・Built-in 版は周りの色の映り込みだけ
- アンチエイリアスの方式 (HDRP は時間方向、URP・Built-in は MSAA)。輪郭のなめらかさが少し違う
- 影の縁のやわらかさ、遠くのぼけ・にじみの広がり方 (同じ強さだが計算の仕方が違う)
- センサ画像は 3 版で近いが、画素までは一致しない。画像で走る方策・検出器は、学習・評価したのと同じ版で動かす

絵作りを変えたら、3 版を同じ位置で撮って並べる:

```bash
./scripts/shots3.sh minicar          # → shots/cmp3_minicar_<日時>/sheet.png (左から HDRP・URP・Built-in)
./scripts/shots3.sh fuji rx7
```

合わせ込みのときは、ビルドし直さずに数値を試せる: プレイヤーに `-tune "sun=1.8,amb=1.4"` (名前は `RenderCompat.Tune` を引いている所)。
決まった値はコードの既定値に入れる。

- 選び方の境目は `-quality auto` (low / medium / high) と同じ。選んだ版のプレイヤーが無ければ 1 つ軽い版へ下げる。選んだ理由は標準エラーに出る
- 指定するとき: `JETRACER_PIPELINE=builtin|urp|hdrp` (または `PLAYER=<実行ファイル>`)
- 自動が既定なのは `tools/race/race3.sh`・`tools/race/race3_fuji.sh` だけ。`race3.sh` で方策を画像で走らせるとき (`BLUE_MODEL`) は Built-in に固定
- launch は `unity_player:=auto` と書いたときだけ自動 (`mlagents:=true` のときは Built-in)。リポジトリが `~/jetracer/jetracer_sim` 以外にあるときは `JETRACER_SIM_ROOT` を渡す

## 画質の段階 `-quality low | medium | high | auto`

起動引数 (launch・`scripts/shots.sh` は `QUALITY=`)。既定 `auto` は GPU を見て選ぶ (`RenderQuality.cs`)。選んだ段階と理由は
起動ログ `[RenderQuality]` と `bench.md` に出る。中身は 3 版で同じ。

| 段階 | 中身 | auto で選ばれる GPU |
|---|---|---|
| low | 後処理なし。富士の影 150 m・森の影なし | ソフトウェア描画・内蔵 GPU (Intel・AMD APU)・ビデオメモリ 2 GB 未満。**`-mlagents` のときは常に low** |
| medium | 後処理あり (下)。富士の影 300 m | ビデオメモリ 2〜6 GB (例: Radeon RX 5300M 3 GB = 評価用の MSI Bravo 15) |
| high | medium + 富士の影 600 m | ビデオメモリ 6 GB 以上 |

後処理 (medium 以上・表示用のカメラだけ):

| 効果 | 富士 | ミニカーの会場 |
|---|---|---|
| トーンカーブ | ACES | ACES |
| にじみ (ブルーム) | 0.12 | 0.22 |
| 物の際の陰り (AO) | 0.6 | 1.0 |
| 遠くのぼけ | なし | 4.5 m から始まり 22 m で最大 (真上からの全景には掛けない) |
| 周辺減光 | なし | 0.22 |
| コントラスト・彩度 | なし | +10・+6 |

配信するセンサ画像 (車載カメラ) は段階によらず同じ (AA なし・後処理は実カメラ風の SensorPost だけ)。
以前 HDRP 版の high にだけあった立体の雲・立体の霧は、3 版で同じ絵にするため外した。

ノート PC (内蔵 + 単体 GPU) では、launch と `scripts/shots.sh` が `DRI_PRIME=1` を既定にして単体 GPU を使う。
基準の数値は [render_baseline.md](render_baseline.md)。

評価用 PC (RX 5300M) での速さ (`-quality medium`・3 分割の画面・1920×1080・計測 600 フレーム、2026-10-06。静かな状態):

| コース | Built-in | URP | HDRP |
|---|---|---|---|
| 富士 | 134 fps | 124 fps | 39 fps |
| ミニカーの会場 (URP・HDRP は部屋、Built-in は地面と空だけ) | 192 fps | 224 fps | 114 fps |

Built-in のミニカーの会場は low で 197 fps。部屋を描いていたときは 123 fps で、いちばん重かったのは壁板の足 (約 400 部品) だった。
学習の 1 判断あたりの時間 (lockstep) は、待ちを直してミニカーの会場 75 ms・富士 98 ms (直す前は 179 ms・143 ms。描画ではなく画像と IMU の待ちが原因だった)。くわしくは [render_baseline.md](render_baseline.md)。

## URP 版のプロジェクト

HDRP 版と同じ方式で、**別のプロジェクト `unity/MinicarSimURP` をスクリプトで作る** (リポジトリには入れない)。スクリプト定義 `MINICAR_URP` を付けて `RenderCompat.Urp.cs` を有効にする。

```bash
./scripts/migrate_urp.sh              # 同期 → ① URP パッケージ → ② 設定 → ③ ビルド → ~/jetracer/unity/player_urp に配置
STEPS="3 install" ./scripts/migrate_urp.sh   # ソースを直したあと (sync をやり直したら 1・2 も要る)
```

`migrate_urp.sh` と `migrate_hdrp.sh` の中身は共通の `scripts/migrate_pipeline.sh <urp|hdrp>`。

- 設定 (`Editor/UrpSetup.cs`): 設定アセット `Assets/MinicarURP/MinicarURP.asset` と描画器、色空間 Linear、HDR・深度テクスチャ・影 4096、`Mat_URPLit` と材質の変形 (`Resources/URPVariants`)。描画 API は Built-in 版と同じ (この PC では OpenGLCore)
- 材質: Standard → Universal Render Pipeline/Lit。車体の塗装はクリアコートつきの Complex Lit (Built-in の 2 枚目のクリア層は外す)。
  ★材質の変形はキーワードだけでなく設定値・テクスチャも入れて保存する (キーワードだけだと取り込み時に消えて、木が黒い板になった)
- 光: 3 版とも Linear。Built-in は光の強さを sRGB の値として読む (強さ 1.8 は 3.6 倍に効く) ので、URP では 2.2 乗して同じ明るさにする
- 影・アンチエイリアス: コードが QualitySettings に入れた値を設定アセットへ写す
- 後処理 (medium 以上・表示用のカメラだけ): 上の表の値を URP の Volume に入れる。際の陰りは描画器の SSAO (phase 2 が足す)。
  センサカメラは SSAO の無い 2 番目の描画器で描く
- ★detail (地面の細かい模様) の絵は Linear の絵として持ち直す。sRGB のままだと URP/Lit は 0.5 を 0.21 と読み、芝が半分以下の暗さになった
- 地面の detail は URP/Lit では 1 番目の UV にしか乗らない (Built-in は 2 番目 = m 単位)。同じ大きさになるようタイルを換算して入れる

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
- 富士: 太陽 100,000 lux・物理的な空・露出 EV100 12.9・霞は指数の霞と同じ減衰で、高さ 2.5 km で薄れる。
  空からの環境光は 3 倍にしてある (物理的な空のままだと日陰が黒くつぶれた。2026-10-05)
- ミニカーの会場: 光の強さは共通の値のまま、露出 EV100 −1.9 で換算。環境光は上・横・下の 3 色の空
- センサカメラには後処理 (トーンマップ・ブルーム) と AO を掛けない。HDRP 既定の Volume が入れる AO・ブルーム・モーションブラーは low で 0、モーションブラーは常に 0
- エピソードごとに作られる床テープ・観戦者も `RenderCompat.Refresh` で置き換える

### ミニカーの会場 (部屋)

ミニカーの会場は、URP 版・HDRP 版では実際にコースを組んだ部屋に寄せて描く (`CourseBuilder.Room.cs`)。
Built-in 版 (学習 `-mlagents` もこの版) は部屋を描かず、コースの周りは平らな地面と空だけ。下の表の「部屋」「備品」以外は Built-in 版にもある。
参考にしたのは会場の動画と「△3 コース・レギュレーション」の使用部材。

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
| 光と後処理 | 窓と蛍光灯のある明るい部屋の光。medium 以上で際の陰り・遠くのぼけ・周辺減光 (上の表) |

配信するセンサ画像にも部屋が写る。

### 注意

- HDRP は Linux では Vulkan が要る。AMD は Mesa (RADV)、NVIDIA は独自ドライバで動く。内蔵 GPU やビデオメモリ 4 GB 未満では重い (Built-in 版の medium を使う)
- 評価用 PC (Unity 6000.0.83f1・HDRP 17・RX 5300M) で全段が通ることを確かめた (2026-10-05)。直したのは Global Settings の型の参照 1 か所と露出 (`kCircuitEV` 14.4 → 12.9)。速さは [render_baseline.md](render_baseline.md)。③ でエラーが出たら `RenderCompat.Hdrp.cs` か `Editor/HdrpSetup.cs` を直す。
  Global Settings の警告が出たら、エディタで `unity/MinicarSimHDRP` を開き Window > Rendering > HDRP Wizard の Fix All を 1 回押して、`STEPS="3 install"` で続ける
- 元に戻す: `unity/MinicarSimHDRP` と `~/jetracer/unity/player_hdrp` を消すだけ (元のプロジェクトは変わっていない)
