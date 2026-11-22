# Run each block separately from the repository root on the robot PC (NUC), project env active.
# start rt-check
uname -v                      # a PREEMPT_RT kernel prints "PREEMPT_RT" (or "PREEMPT RT")
cat /sys/kernel/realtime      # 1 = realtime kernel; the file is missing on a normal kernel
ulimit -r                     # rtprio limit of this shell; 99 after the limits step
# end rt-check
# start rt-limits
sudo groupadd realtime
sudo usermod -aG realtime "$USER"
printf '@realtime soft rtprio 99\n@realtime soft priority 99\n@realtime soft memlock 102400\n@realtime hard rtprio 99\n@realtime hard priority 99\n@realtime hard memlock 102400\n' | sudo tee -a /etc/security/limits.conf
# Log out and in again (or reboot), then rerun the rt-check block.
# end rt-limits
# start server-version
rwrl doctor
# end server-version
# start franky-version
python -c 'import importlib.metadata as m; print(m.version("franky-control"))'
# end franky-version
# start install-wheel
rwrl doctor --install-sdk
# end install-wheel
# start install-native
LIBFRANKA_VERSION=0.17.0 bash native/serl_controller/scripts/install-on-ubuntu.sh
# end install-native
# start install-native-check
python -c 'import franky, serl_controller as sc; print(sc.build_info())'
cat ~/.cache/rwrl-native-build/BUILD_INFO.txt
# end install-native-check
# start backup
cp local/station.yaml local/station.yaml.before-controller.bak
# end backup
# start setup-switch
rwrl setup --station local/station.yaml --non-interactive --cartesian-controller serl --robot-model fr3 --realtime enforce
# end setup-switch
# start doctor
rwrl doctor
# end doctor
# start home
rwrl home show
rwrl home go
# end home
# start recover
rwrl recover
rwrl status
# end recover
# start rollback
cp local/station.yaml.before-controller.bak local/station.yaml
rwrl doctor
# end rollback
# start wheels-docker
# Optional, on a build machine with Docker (not the robot PC): a wheel pair for identical robot PCs.
WORK=$HOME/rwrl-franky-build LIBFRANKA_VERSION=0.17.0 native/serl_controller/docker/build-on-host.sh
ls $HOME/rwrl-franky-build/out/wheels
# end wheels-docker
# start wheels-install
# On the robot PC, with the whole wheel directory copied over:
W=path/to/wheels
(cd $W && sha256sum -c SHA256SUMS)
pip uninstall -y franky-control serl-controller
pip install $W/franky_control-*.whl $W/serl_controller-*.whl
# end wheels-install
