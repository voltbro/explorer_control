source "${ROS_DIR}/devel/setup.bash"
source "${ROS_DIR}/install/setup.bash"

function cmd-vel() {
    rostopic pub /cmd_vel -r ${3:-10} geometry_msgs/Twist "linear:
    x: ${1:-0.0}
    y: 0.0
    z: 0.0
angular:
    x: 0.0
    y: 0.0
    z: ${2:-0.0}"
}
