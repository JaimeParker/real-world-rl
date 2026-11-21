Minimal stand-ins for the ROS / franka_ros headers included by the vendored,
unmodified SERL controller in ../serl_original/. They provide just enough API
for that source to compile and let the golden test drive init() / starting() /
update() and the two callbacks deterministically. They contain no control math.

The one exception is `ros/time.h`: SERL's JointPositionController keeps its
blend clock in a `ros::Duration` (`elapsed_time_ += period`, then
`elapsed_time_.toSec()`), so `ros::Duration` reproduces roscpp_core's rostime
arithmetic verbatim (int32 sec + int32 nsec, `fromSec()` rounding, `operator+`
via nanoseconds, `normalizeSecNSecSigned()`, `toSec()`), from
ros/roscpp_core noetic-devel @ a1a1942 (BSD, Willow Garage).
