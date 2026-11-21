# Run blocks in order after editing local/station.yaml for the actual hardware.
# start copy
mkdir -p local
cp site/examples/station.yaml local/station.yaml
cp site/examples/task.yaml local/task.yaml
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
# start workspace
rwrl --station local/station.yaml workspace show
# end workspace
# start gripper-state
rwrl --station local/station.yaml gripper show
# end gripper-state
# start camera-state
rwrl --station local/station.yaml camera show --name wrist
# end camera-state
