#!/usr/bin/env bash
set -euo pipefail

if [[ "${CONDA_DEFAULT_ENV:-}" != "vtol_nav" ]]; then
    echo "Activate the vtol_nav environment first." >&2
    exit 1
fi

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
export PYTHONPATH="${project_root}/src${PYTHONPATH:+:${PYTHONPATH}}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

python - <<'PY'
import sys

import cv2
import gymnasium
import numpy
import stable_baselines3
import torch
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import ca_lsc_td3
from ca_lsc_td3._vendor.torch_cfc import Cfc

assert sys.version_info[:2] == (3, 10), sys.version
assert numpy.__version__ == "1.26.4", numpy.__version__
assert cv2.__version__ == "4.11.0", cv2.__version__
assert gymnasium.__version__ == "1.3.0", gymnasium.__version__
assert stable_baselines3.__version__ == "2.9.0", stable_baselines3.__version__
assert torch.__version__.split("+", maxsplit=1)[0] == "2.13.0", torch.__version__

message = Image()
message.height = 1
message.width = 1
message.encoding = "rgb8"
message.step = 3
message.data = bytes([1, 2, 3])
converted = CvBridge().imgmsg_to_cv2(message, "bgr8")
assert converted.shape == (1, 1, 3)
assert converted.tolist() == [[[3, 2, 1]]]

assert ca_lsc_td3.smoothstep(0.5, 0.0, 1.0) == 0.5
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
value = torch.arange(1024, device=device, dtype=torch.float32).sum().item()
assert value == 523776.0

cfc = Cfc(
    12,
    64,
    1,
    {
        "backbone_activation": "silu",
        "backbone_units": 32,
        "backbone_layers": 1,
    },
)
sequence = torch.zeros((2, 20, 12), dtype=torch.float32, device=device)
timespans = torch.full((2, 20), 0.1, dtype=torch.float32, device=device)
cfc = cfc.to(device)
cfc_output = cfc(sequence, timespans)
assert cfc_output.shape == (2, 1)
assert torch.isfinite(cfc_output).all()

print(f"ca_lsc_td3={ca_lsc_td3.__version__}")
print(f"python={sys.version.split()[0]}")
print(f"numpy={numpy.__version__}")
print(f"opencv={cv2.__version__}")
print(f"gymnasium={gymnasium.__version__}")
print(f"stable_baselines3={stable_baselines3.__version__}")
print(f"torch={torch.__version__} device={device}")
print(f"cfc_forward=ok shape={tuple(cfc_output.shape)}")
print("cv_bridge_rgb_conversion=ok")
print("core_environment=ok")
PY
