# Run each block separately from the repository root.
# start hid
python -c 'import hid; print(hid.enumerate())'
ls -l /dev/hidraw*
id
# end hid
# start read
rwrl teleop read --duration-s 10 --rate-hz 10
rwrl teleop diagnose --duration-s 5
# end read
# start calibrate
rwrl teleop calibrate --backend spacemouse --profile operator-base --frame base --output spacemouse-profiles.yaml --use
rwrl teleop profiles show --profile operator-base
rwrl workspace show
# end calibrate
# start calibrate-table
rwrl teleop calibrate --backend spacemouse --profile operator-table --frame table --output spacemouse-profiles.yaml --use
rwrl teleop profiles show --profile operator-table
rwrl workspace frame show
# end calibrate-table
# start axis-check
rwrl workspace capture
rwrl teleop run --backend spacemouse --record local/spacemouse-axis --num-successes 1 --max-episodes 1
rwrl workspace capture
# end axis-check
# start serl-axis-check
rwrl workspace capture
rwrl --preset local/task-serl.yaml teleop run --backend spacemouse --mode serl --record local/spacemouse-serl-axis --num-successes 1 --max-episodes 1
rwrl workspace capture
# end serl-axis-check
# start practice
rwrl teleop run --backend spacemouse --record local/spacemouse-practice --num-successes 1 --max-episodes 1
# end practice
# start serl
rwrl --preset local/task-serl.yaml teleop run --backend spacemouse --mode serl --record local/spacemouse-serl --num-successes 1 --max-episodes 1
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
rwrl tcp
# Convert its xyzw quaternion using the same Euler convention as the implementation.
python -c 'from scipy.spatial.transform import Rotation; q = [float(x) for x in input("quat xyzw (four space-separated values): ").split()]; rpy = Rotation.from_quat(q).as_euler("xyz"); print("abs(roll), pitch, yaw (rad):", [abs(rpy[0]), rpy[1], rpy[2]])'
# end serl-copy

# start probe
rwrl teleop probe-hid --duration-s 10
# end probe

# start profiles-select
rwrl teleop profiles list
rwrl teleop profiles use --profile operator-base
rwrl teleop profiles show --profile operator-base
# end profiles-select
