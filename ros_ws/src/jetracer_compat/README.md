# jetracer_compat — JetRacer 標準のコードを無改造で sim につなぐ

NVIDIA JetRacer (https://github.com/NVIDIA-AI-IOT/jetracer) のままの車両ソフト
(`NvidiaRacecar` に steering / throttle を −1〜1 で書き、`jetcam` の `CSICamera` から画像を読む) を、
**コードを変えずに** この sim で走らせて評価するための互換クラス。ROS を使っていないコードでも動く。

```python
import jetracer_compat
jetracer_compat.install()          # ← この 2 行を先頭に足すだけ

from jetracer.nvidia_racecar import NvidiaRacecar   # 以下は元のコードのまま
from jetcam.csi_camera import CSICamera
car = NvidiaRacecar()
camera = CSICamera(width=224, height=224, capture_fps=65)
car.steering_gain = -0.55; car.steering_offset = 0.12
car.throttle = 0.15
```

## 変換 (実機と同じ)

| 車両ソフトが書く値 | 実機 | sim |
|---|---|---|
| `car.steering` (−1〜1) | u = 値 × steering_gain + steering_offset → PCA9685 のパルス 1500 + 750·u µs → サーボ | 同じパルスを `jetracer_bridge.yaml` の舵の較正で δ [rad] に戻し `/actuator_cmd` へ |
| `car.throttle` (−1〜1) | u = 値 × throttle_gain → パルス → ESC | 同じパルスを `throttle.map_v_us` の逆で v [m/s] に (中立より下は後退。sim の ESC 模型が中立経由を再現) |
| `camera.value` / `read()` | CSI カメラ 224×224 BGR | sim の `/camera/image_raw` (実機の取り込みに合わせた画角・縦横比・歪み) |

較正は実機ブリッジと同じファイル (`jetracer_bridge/config/jetracer_bridge.yaml`) を読むので、実機を較正すれば sim も同じ値になる。
★ いまのスロットル写像 `map_v_us` は仮値 (0.15 → 1.2 m/s、0.2 → 2.0 m/s)。実機で測って入れること。
指令は 30 Hz で再送する (実機の PCA9685 はパルスを保持し続けるため)。プログラムが止まると sim 側は 300 ms で失効して止まる。

## 動かし方

```bash
# sim PC
source scripts/sim_env.sh
ros2 launch minicar_sim sim_host.launch.py unity_player:=$HOME/jetracer/unity/player/MinicarSim.x86_64 tcp_port:=10001
# 車両ソフトを動かす側 (Jetson でも PC でも。ROS_DOMAIN_ID を揃える)
source scripts/sim_env.sh
jupyter lab            # この端末から起動したノートブックで import jetracer_compat が使える
ros2 run jetracer_compat jetracer_compat_demo --throttle 0.15 --steering 0.3   # 動作確認 (右に曲がる)
```
依存: `traitlets` (JetRacer の環境には入っている。無ければ `sudo apt install python3-traitlets`)。
