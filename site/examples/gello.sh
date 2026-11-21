# Run blocks separately; replace serial path and IDs with the inventory first.
# start serial
ls -l /dev/serial/by-id/
id
python -c 'import evdev; print([(p, evdev.InputDevice(p).name) for p in evdev.list_devices()])'
# end serial
# start dialout
sudo usermod -aG dialout "$USER"
# end dialout
# start calibrate
# 1..7 and 8 are an example inventory, not automatic discovery.
rwrl --station local/station.yaml teleop calibrate --backend gello --port /dev/serial/by-id/REPLACE_WITH_GELLO --joint-ids 1 2 3 4 5 6 7 --gripper-id 8 --baudrate 57600 --use
# end calibrate
# start read
rwrl --station local/station.yaml teleop read --duration-s 10 --rate-hz 10
rwrl --station local/station.yaml q
# end read
# start practice
rwrl --station local/station.yaml --preset local/task.yaml teleop run --backend gello --record local/gello-practice.pkl --num-successes 1 --max-episodes 1
# end practice
# start joint
# First copy the session's resolved-config.yaml to local/gello-joint.yaml and edit it.
python -m real_world_rl.demos --config local/gello-joint.yaml --output-path local/gello-joint.pkl --num-successes 1 --max-episodes 1
# end joint
# start bimanual
cp examples/presets/bimanual_gello_smoke.yaml local/gello-bimanual.yaml
# After configuring and starting both station daemons, and reviewing the copied preset:
python -m real_world_rl.demos --config local/gello-bimanual.yaml --output-path local/gello-bimanual.pkl --num-successes 1 --max-episodes 1
# end bimanual
