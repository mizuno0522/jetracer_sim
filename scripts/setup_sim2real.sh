#!/bin/bash
# sim→real 画像変換 (tools/sim2real) 用の Python 環境を作る。ROS の環境 (apt の numpy/cv2) を汚さないよう専用 venv。
#   ./scripts/setup_sim2real.sh            # 既定 ~/jetracer/venv_sim2real。CPU 版 PyTorch
#   TORCH_INDEX=https://download.pytorch.org/whl/cu128 ./scripts/setup_sim2real.sh   # NVIDIA GPU の PC
# Jetson では JetPack 同梱の PyTorch を使う (この venv は作らない)。
set -e
VENV="${SIM2REAL_VENV:-$HOME/jetracer/venv_sim2real}"
INDEX="${TORCH_INDEX:-https://download.pytorch.org/whl/cpu}"
if [ ! -x "$VENV/bin/python" ]; then
  # python3 -m venv は python3.10-venv (apt・sudo) が要る。無ければ pip の virtualenv で作る
  if ! python3 -m venv "$VENV" 2>/dev/null; then
    rm -rf "$VENV"
    command -v virtualenv >/dev/null || python3 -m pip install --user -q virtualenv
    PATH="$HOME/.local/bin:$PATH" virtualenv -q -p python3 "$VENV"
  fi
fi
"$VENV/bin/pip" install -q --upgrade pip
"$VENV/bin/pip" install -q torch --index-url "$INDEX"
# numpy<2: rclpy / sensor_msgs と同じ ABI 系に揃える。onnxruntime は推論ノード (ROS) 用
"$VENV/bin/pip" install -q "numpy<2" pillow onnx onnxscript onnxruntime pyyaml
"$VENV/bin/python" -c "import torch, onnxruntime, numpy; print('torch', torch.__version__, 'cuda', torch.cuda.is_available(), '| onnxruntime', onnxruntime.__version__, '| numpy', numpy.__version__)"
echo "venv: $VENV"
