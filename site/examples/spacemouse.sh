# Run each block separately from the repository root.
# start hid
python -c 'import hid; print(hid.enumerate())'
ls -l /dev/hidraw*
id
# end hid
# start read
rwrl --station local/station.yaml teleop read --duration-s 10 --rate-hz 10
rwrl --station local/station.yaml teleop diagnose --duration-s 5
# end read
# start calibrate
rwrl --station local/station.yaml teleop calibrate --backend spacemouse --profile operator-base --frame base --output spacemouse-profiles.yaml --use
rwrl --station local/station.yaml teleop profiles show --profile operator-base
rwrl --station local/station.yaml workspace show
# end calibrate
# start practice
rwrl --station local/station.yaml --preset local/task.yaml teleop run --backend spacemouse --record local/spacemouse-practice.pkl --num-successes 1 --max-episodes 1
# end practice
# start serl
rwrl --station local/station.yaml --preset local/task-serl.yaml teleop run --backend spacemouse --mode serl --record local/spacemouse-serl.pkl --num-successes 1 --max-episodes 1
# end serl
# start help
rwrl teleop read --help
rwrl teleop diagnose --help
rwrl teleop calibrate --help
rwrl teleop profiles show --help
rwrl teleop run --help
# end help
# start serl-copy
cp local/task.yaml local/task-serl.yaml
# Read TCP at each independently reviewed safe orientation, with control stopped.
rwrl --station local/station.yaml tcp
# Convert its xyzw quaternion using the same Euler convention as the implementation.
python -c 'from scipy.spatial.transform import Rotation; q = [float(x) for x in input("quat xyzw (four space-separated values): ").split()]; rpy = Rotation.from_quat(q).as_euler("xyz"); print("abs(roll), pitch, yaw (rad):", [abs(rpy[0]), rpy[1], rpy[2]])'
# end serl-copy
