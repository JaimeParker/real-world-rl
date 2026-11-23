# Run blocks in order after editing local/station.yaml for the actual hardware.
# start copy
mkdir -p local
cp docs/examples/station.yaml local/station.yaml
cp docs/examples/task.yaml local/task.yaml
# end copy
# start discover
rwrl discover
# end discover
# start doctor
rwrl doctor --station local/station.yaml
# end doctor
# start arm-state
rwrl --station local/station.yaml status
rwrl --station local/station.yaml q
rwrl --station local/station.yaml tcp
# end arm-state
# start corner
rwrl --station local/station.yaml tcp
# end corner
# start limits
rwrl --station local/station.yaml limits show
# end limits
# start rebase
rwrl limits rebase local/station.yaml
rwrl limits rebase local/station.yaml --write
# end rebase
# start workspace
rwrl --station local/station.yaml workspace show
# end workspace
# start gripper-state
rwrl --station local/station.yaml gripper show
# end gripper-state
# start camera-state
rwrl --station local/station.yaml camera show --name wrist
# end camera-state
# start use
rwrl use station local/station.yaml
rwrl use preset local/task.yaml
# end use
# start use-show
rwrl use
rwrl workspace show
# end use-show
# start frame-sign
# Before: TCP somewhere in the middle of the workspace. Then guide it straight up ~10 cm (vertical in the room).
rwrl tcp
rwrl tcp
# end frame-sign
# start frame-capture
# Guide the TCP to each point (same height above the table), stop guiding, then capture:
rwrl workspace capture --name frame-origin --output frame-points.yaml --save
rwrl workspace capture --name frame-x --output frame-points.yaml --save
rwrl workspace capture --name frame-y --output frame-points.yaml --save
# end frame-capture
# start frame-set
rwrl workspace frame set --name table --from-points frame-points.yaml --origin frame-origin --x-axis frame-x --y-axis frame-y
rwrl workspace frame set --name table --from-points frame-points.yaml --origin frame-origin --x-axis frame-x --y-axis frame-y --save
rwrl workspace frame show
# end frame-set
# start setup
rwrl setup --arms 1 --arm-names franka --robot-ip 172.16.0.2 --robot-model fr3 --scope local
rwrl setup --help
# end setup
# start setup-noninteractive
rwrl setup --station local/station.yaml --only cameras --non-interactive --cameras none --output local/station.no-cameras.yaml
# end setup-noninteractive
# start setup-calibrate
rwrl setup --station local/station.yaml --only teleop --calibrate
# end setup-calibrate
# start setup-mock
rwrl setup --mock --output local/station.mock.yaml
# end setup-mock
# start setup-bimanual
rwrl setup --arms 2 --arm-names left right --robot-ip 172.16.0.2 172.16.0.4 --daemon-port 50051 50052 --output local/station.bimanual.yaml
rwrl doctor --station local/station.bimanual.yaml --robot left
rwrl doctor --station local/station.bimanual.yaml --robot right
# end setup-bimanual
# start network-check
ip -br -4 addr
ip route get 172.16.0.2
ping -c 3 -W 1 172.16.0.2
ROBOT_IF=enp86s0  # Replace with the interface wired to Control.
# end network-check
# start network-temporary
sudo ip addr add 172.16.0.1/24 dev "$ROBOT_IF"
sudo ip link set "$ROBOT_IF" up
ping -I "$ROBOT_IF" -c 3 -W 1 172.16.0.2
# end network-temporary
# start network-persistent
ROBOT_CONNECTION="$(nmcli -g GENERAL.CONNECTION device show "$ROBOT_IF")"
if [ -z "$ROBOT_CONNECTION" ] || [ "$ROBOT_CONNECTION" = "--" ]; then
  ROBOT_CONNECTION=franka-control
  sudo nmcli connection add type ethernet ifname "$ROBOT_IF" con-name "$ROBOT_CONNECTION"
fi
sudo nmcli connection modify "$ROBOT_CONNECTION" \
  ipv4.method manual ipv4.addresses 172.16.0.1/24 ipv4.gateway "" \
  ipv4.never-default yes connection.autoconnect yes
sudo nmcli connection up "$ROBOT_CONNECTION"
ip route get 172.16.0.2
# end network-persistent
# start doctor-fix
rwrl doctor --station local/station.yaml --fix
# end doctor-fix
# start home-preset
rwrl --station local/station.yaml --preset local/task.yaml home set
rwrl --station local/station.yaml --preset local/task.yaml home show
# end home-preset
# start camera-grab
rwrl --station local/station.yaml camera grab --name wrist --output local/camera-wrist
# end camera-grab
