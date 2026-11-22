# Blocks are independent tutorial steps, not an unattended installation script.
# start checkout
git clone --recurse-submodules https://github.com/JaimeParker/real-world-rl.git
cd real-world-rl
git rev-parse HEAD
git submodule status
# end checkout
# start direct
bash scripts/install.sh --extras learning,franky,realsense,spacemouse
# end direct
# start daemon
bash scripts/install.sh --extras franky,grpc,realsense
# end daemon
# start client
bash scripts/install.sh --extras learning,grpc,spacemouse
# end client
# start learner
bash scripts/install.sh --extras learning
# end learner
# start activate
source .venv/bin/activate
# end activate
# start verify
python -c 'import real_world_rl, real_world_rl.server; print("Imports OK:", real_world_rl.__file__)'
rwrl --help
rwrl doctor --mock --station docs/examples/mock-station.yaml
# end verify
# start learning-import
python -c 'import rl_garden, real_world_rl; print("Imports OK:", rl_garden.__file__, real_world_rl.__file__)'
# end learning-import
# start check-franky
python -c 'import franky; print("franky OK")'
# end check-franky
# start check-realsense
python -c 'import pyrealsense2; print("RealSense OK")'
# end check-realsense
# start check-zed-uvc
python -c 'import cv2; print(cv2.__version__)'
# end check-zed-uvc
# start check-zed
python -c 'import pyzed.sl; print("ZED SDK OK")'
# end check-zed
# start check-spacemouse
python -c 'import hid; print(hid.enumerate())'
# end check-spacemouse
# start check-gello
python -c 'import dynamixel_sdk; print("Dynamixel SDK OK")'
# end check-gello
# start check-events
python -c 'import evdev; print(evdev.list_devices())'
# end check-events
# start check-pico
python -c 'import zmq; print(zmq.zmq_version())'
# end check-pico
# start check-keyboard
python -c 'import pynput; print("keyboard OK")'
# end check-keyboard
# start check-grpc
python -c 'import grpc; print(grpc.__version__)'
# end check-grpc
# start check-jpeg
python -c 'import PIL, cv2; print("JPEG dependencies OK")'
# end check-jpeg
# start check-openpi
python -c 'import websockets; print(websockets.__version__)'
# end check-openpi
# start check-terminal
tty
# end check-terminal
# start manual
unset PYTHONPATH
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -e ./rl-garden -e '.[learning,franky,realsense,spacemouse,test]'
# end manual
