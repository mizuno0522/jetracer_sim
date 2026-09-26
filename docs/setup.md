# 環境構築 — 2 ホスト (sim PC ＋ Jetson Orin Nano)

| ホスト | OS / ROS | 役割 | IP (有線直結・ゲートウェイ無し) | chrony |
|---|---|---|---|---|
| sim PC | Ubuntu 22.04.5・AMD Ryzen・**AMD GPU (CUDA 無し)** / Humble | Unity (素) ＋ 物理・imu_sim・endpoint (素か Docker)。学習器は別 PC | 192.168.10.2 (有線 `enp5s0`) | server |
| Jetson Orin Nano | JetPack 6.2.1 (L4T 36.x・Ubuntu 22.04) / Humble | 推論・ブリッジ・カメラ | 192.168.10.3 (有線 `enP8p1s0`) | client |

各ステップの最後の「確認」が通らないうちは次へ進まない。**P9 (Jetson ↔ PC の実接続) はまだ誰も試していない**ので、Step 3 を最初に通す半日を取ること。

## Step 0 — 全ホスト: 一気に作る (`scripts/setup_host.sh`)

apt (Humble・cyclonedds・cv_bridge・rosbag2・chrony)・sysctl・固定 IP・chrony・`~/cyclonedds.xml`・`~/.bashrc` を 1 本で。何度実行しても同じ状態になる。

```bash
git clone https://github.com/mizuno0522/jetracer_sim.git ~/jetracer/jetracer_sim && cd ~/jetracer/jetracer_sim
sudo ./scripts/setup_host.sh jetson          # Jetson。sim PC は  sudo ./scripts/setup_host.sh pc
source ~/.bashrc
```

- 有線 NIC は自動検出 (`nmcli device status` の最初の ethernet)。違うときは `--nic enp5s0`。
- 固定 IP と chrony を触りたくないとき (1 台で使うだけ) は `--no-net`。
- DDS 設定は `~/cyclonedds-wired.xml` (有線限定) と `~/cyclonedds-local.xml` (指定なし) の 2 枚を作り、`source scripts/sim_env.sh` が有線のリンク状態で自動で選ぶ。有線限定のままケーブルを抜くと 1 台構成でも探索が失敗するため。
- 接続名は `jetracer-link`。既存の「有線接続 1」は残るが、autoconnect の優先度で `jetracer-link` が勝つ。戻すには `nmcli con delete jetracer-link`。
- 手で 1 つずつやる場合の内容はスクリプトの先頭コメント (旧 Step 0〜1 と同じ)。ROS の apt ソース登録が公式の変更で詰まったら docs.ros.org の Humble のページを見る。

**確認**:

```bash
ros2 doctor --report | grep -iE "rmw|domain"   # rmw_cyclonedds_cpp と 42
sysctl net.core.rmem_max                       # 16777216
ip -br addr | grep 192.168.10.                 # 有線に .2 / .3
```

## Step 1 — 各ホスト: ワークスペース

```bash
# sim PC (ros_tcp_endpoint を deps.repos から取り込む。Unity プレイヤーは既存ビルドのコピー + course.json)
./scripts/setup_ws.sh && ./scripts/build.sh && ./scripts/test.sh
./scripts/setup_unity_player.sh                 # → ~/jetracer/unity/player   (docs/unity.md)
# Jetson (endpoint は要らない)
./scripts/build.sh && ./scripts/test.sh
cat /etc/nv_tegra_release                       # R36 (release), REVISION: 4.x
```

**確認**: `./scripts/smoke_test.sh` (1 台・Unity 無しの閉ループ) が両ホストで通る。Docker を使うなら [docs/docker.md](docker.md) (AMD GPU の PC では sim コンテナのみ)。

## Step 2 — ケーブルを直結して時刻を合わせる

1000BASE-T で直結 (スイッチを挟まない)。両ホストで:

```bash
./scripts/p9_check.sh      # IP・ping・RMW・cyclonedds.xml の NIC・rmem_max・chrony・DOMAIN_ID
```

**確認** (Jetson で): `chronyc tracking` の Reference が 192.168.10.2、System time のずれが 1 ms 未満。数十 ms 以上ずれていると鮮度判定が全部弾く (既存記録で確認済みの構図)。起動直後は数分待つ。

## Step 3 — 2 ホスト接続 (P9)

**2026-09-26 に通った** (sim PC: Ryzen/AMD GPU・Humble 素、Jetson Orin Nano、1000BASE-T 直結、cyclonedds 有線限定、DOMAIN 42): PC の sim (OpenCV 描画) → Jetson で /imu 99 Hz・/camera/image_raw 14.4〜15 Hz・/sim/ground_truth 99 Hz を受信、Jetson の教師スタック → PC で /actuator_cmd 30 Hz。閉ループは 1 台のときと同じ 24.0 s/周・衝突 0・cte p95 0.10 m (`tools/lap_eval.py` を Jetson で実行)。Jetson での受信間隔 (30 s・`ros2 topic hz`): 画像 15.0 Hz・最大 87 ms、/imu 99.7 Hz・最大 44 ms、failsafe の ESTOP 0 回 (最初に見えた 0.67 s の抜けは計測用の Python 購読者の取りこぼしで、経路の問題ではなかった)。PC 側 (送り元) は画像 publish→受信 1.5 ms、/imu 8 ms (imu_sim の遅延モデル込み)。chrony 同期後 (ずれ 0.1 ms) の遅延 (stamp → Jetson 受信、30 s・`ros2 topic delay`):

| トピック | 平均 | 最大 | 備考 |
|---|---|---|---|
| /camera/image_raw (OpenCV 描画) | 5 ms | 33 ms | stamp は描画後なので描画時間は含まない。Unity 描画は stamp = 姿勢の sim 時刻なので描画時間込みになる (要別測) |
| /imu | 12 ms | 44 ms | imu_sim の遅延モデル (base 4 ms ＋ ジッタ) 込み |
| /sim/ground_truth | 1 ms | 31 ms | |
| **/camera/image_raw (Unity 描画)** | **64 ms** | 90 ms | stamp = 姿勢の sim 時刻。描画 ＋ ROS-TCP-Endpoint ＋ DDS 込み (PC ローカルで 62 ms、有線で +2〜5 ms)。既存記録の 0.07〜0.10 s より少し良い (224×224)。failsafe の 150 ms に余裕 |
| /imu (Unity 時) | 8 ms | 22 ms | |
| /actuator_cmd (Jetson → PC、往路) | 0.8 ms | 1.3 ms | PC 側で計測 |

1 台 (localhost) の既存記録「画像 0.07〜0.10 s」は Unity 描画込みの値で、2 ホストでも 64 ms。有線区間そのものは数 ms で、設計の見積りどおり。Unity 描画・2 ホストでの閉ループも 24.0 s/周・衝突 0・cte p95 0.10 m (OpenCV 描画と同じ = 教師は画像を見ていないので当然。画像を見る policy_net で差が出るかはこれから)。**画像と IMU の相対遅れは Unity で 64 − 8 = 56 ms、OpenCV で 5 − 12 = −7 ms**。学習データは Unity で取る前提なので、実機のカメラ遅延 (露光 → 受信) をこの 56 ms に合わせるか、imu_sim の latency を実機に合わせるかを較正で決める (docs/calibration.md)。**計測は `ros2 topic hz`/`delay` か 1 トピックだけの購読者で。複数トピックを 1 つの spin_once ループで受けると Orin Nano では取りこぼす。**


```bash
# sim PC
ros2 launch minicar_sim sim_host.launch.py camera_backend:=opencv unity_player:=none   # まず Unity 無し
# Jetson (別端末)
ros2 topic hz /imu                  # 100 Hz
ros2 topic hz /camera/image_raw     # 15 Hz。0 なら受信バッファか cyclonedds.xml の NIC
ros2 topic delay /camera/image_raw  # 1 台 (localhost) の 0.07〜0.10 s から数 ms 増える程度なら正常
ros2 launch jetracer_stack vehicle_stack.launch.py teacher:=true auto_run:=true
```

**確認**: sim PC で `ros2 topic hz /actuator_cmd` が 30 Hz (Jetson の指令が届いている)、`python3 tools/lap_eval.py --seconds 60` で周回・衝突 0。通ったら `camera_backend:=unity unity_player:=~/jetracer/unity/player/MinicarSim.x86_64` で同じことをし、`ros2 topic delay /camera/image_raw` を記録する (IMU との相対遅れが学習の当たり外れを決める。設計 IMU 合成タブ)。

## Jetson の画面をネットワーク越しに見る (VNC の仮想ディスプレイ)

物理モニターとは別の画面を Jetson に作り、Mac (VNC Viewer か標準の「画面共有」) から見る。モニターがちらつく・外したいときに。

```bash
sudo ./scripts/setup_vnc.sh          # 導入・ログイン不要の自動起動・パスワード設定 (1 回だけ)
# Mac のターミナル (開いたまま):  ssh -N -L 5901:localhost:5901 jetson@ubuntu.local
# Mac の VNC Viewer:  localhost:5901
```

既定は Jetson の中 (localhost) でしか待ち受けず、Mac からは SSH の暗号化トンネルで入る (VNC のパスワードは平文で流れるため)。家の LAN だけで直接つなぐなら `sudo ./scripts/setup_vnc.sh --lan` → VNC Viewer で `ubuntu.local:5901`。仮想ディスプレイの中の OpenGL (rviz など) はソフトウェア描画で遅い。

## 失敗の定番

| 症状 | 原因 | 直し方 |
|---|---|---|
| 別ホストのトピックが見えない | `ROS_LOCALHOST_ONLY=1` / RMW が両ホストで違う / `cyclonedds.xml` の NIC が無線のまま | `p9_check.sh`。Fast DDS と Cyclone を混ぜない |
| `/imu` は来るが画像だけ来ない | `net.core.rmem_max` が既定 (212992) | `setup_host.sh` の sysctl。Docker の中では効かないのでホストで |
| 走り出してすぐ ESTOP | chrony のずれで鮮度判定が弾く | `chronyc tracking`。ずれ 1 ms 未満まで待つ |
| chrony が同期しない (`chronyc sources` で reach 0) | 相手の ufw が UDP 123 を落としている | `sudo ufw allow from 192.168.10.0/24` (`setup_host.sh` が ufw 有効時に入れる) |
| `rosidl` / `cv2` が壊れる | `~/.local` の pip empy 4.x / numpy 2.x | `PYTHONNOUSERSITE=1` (`.bashrc` に入れてある)。pip 側は消さない |
| Unity 有りで launch ごと止まる | `ros_tcp_endpoint` が無い | sim PC で `./scripts/setup_ws.sh` (deps.repos) |
| インターネットが切れた | 有線にゲートウェイが付いた | `jetracer-link` は `never-default`。`ip route` で default が無線であること |
