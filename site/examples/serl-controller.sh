# Run each block separately from the repository root.
# start rt-check
uname -v                      # a PREEMPT_RT kernel prints "PREEMPT_RT" (or "PREEMPT RT")
cat /sys/kernel/realtime      # 1 = realtime kernel; the file is missing on a normal kernel
ulimit -r                     # rtprio limit of this shell; 99 after the limits.conf step
# end rt-check
# start rt-limits
sudo groupadd realtime
sudo usermod -aG realtime "$USER"
printf '@realtime soft rtprio 99\n@realtime soft priority 99\n@realtime soft memlock 102400\n@realtime hard rtprio 99\n@realtime hard priority 99\n@realtime hard memlock 102400\n' | sudo tee -a /etc/security/limits.conf
# Log out and in again (or reboot), then rerun the rt-check block.
# end rt-limits
# start copy-wheels
# On the development machine that holds the build (local/ is not in git):
rsync -av local/wheels/rwrl-franky-build-20261122-cxxpass ROBOT_PC:REPLACE_WITH_REPO_PATH/local/wheels/
# end copy-wheels
# start install-wheels
python --version              # must be 3.10: the wheels are cp310
W=local/wheels/rwrl-franky-build-20261122-cxxpass
(cd $W && sha256sum -c SHA256SUMS)
pip uninstall -y franky-control serl-controller
pip install $W/franky_control-*.whl $W/serl_controller-*.whl
python -c 'import franky, serl_controller as sc; print(sc.BUILD_INFO)'
# end install-wheels
# start backup
cp local/station.yaml local/station.yaml.before-serl.bak
ls local/station.yaml*.bak
# end backup
# start doctor
rwrl doctor
# end doctor
# start first-servo
rwrl teleop run --backend spacemouse --record local/serl-first.pkl --num-successes 1 --max-episodes 1
# end first-servo
# start serl-mode
rwrl --preset local/task-serl.yaml teleop run --backend spacemouse --mode serl --record local/serl-mode.pkl --num-successes 1 --max-episodes 1
# end serl-mode
# start recover
rwrl recover
rwrl status
# end recover
# start rollback
cp local/station.yaml.before-serl.bak local/station.yaml
rwrl doctor
# end rollback
