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
