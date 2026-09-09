# `vtol_nav` environment

ROS 2 Humble binary packages are built for Python 3.10 and NumPy 1.x.
The project therefore pins NumPy 1.26 and OpenCV 4.11. NumPy 2.x can make
`cv_bridge` color conversion crash the interpreter.

Repair the existing environment:

```bash
conda activate vtol_nav
PYTHONNOUSERSITE=1 python -m pip install \
  -r ~/ca_lsc_td3/environment/requirements-vtol-nav.in
python -m pip check
~/ca_lsc_td3/scripts/check_environment.sh
```

`requirements-vtol-nav.in` contains the verified direct pins used by this
machine. It is deliberately not called a lock file: it does not enumerate
every transitive wheel or capture the CUDA wheel index. Use the existing
`vtol_nav` environment for this project. Before claiming environment-level
reproducibility on another machine, generate a platform-specific lock that
also records the PyTorch CUDA wheel source.

`torch==2.13.0+cu130` is the currently installed CUDA-enabled build. If recreating
the environment on another machine, install the matching CUDA wheel from the
PyTorch index before installing the remaining requirements.

`requirements-px4-1.15.txt` records the Python tools needed to build the pinned
PX4 tree; it is separate from the `vtol_nav` research environment.

ROS packages should be built after deactivating Conda. Training commands are
run inside `vtol_nav` after sourcing ROS and the workspace.
