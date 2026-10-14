#!/usr/bin/env bash
set -e
demo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
px4_dir="${PX4_DIR:-/home/a/Libraries/PX4-Autopilot}"
if [[ ! -x "$px4_dir/build/px4_sitl_default/bin/px4" ]]; then
  echo "PX4 SITL executable missing in $px4_dir; build PX4 first." >&2
  exit 1
fi
source /opt/ros/noetic/setup.bash
if [[ -f "$demo_dir/../../../devel/setup.bash" ]]; then
  source "$demo_dir/../../../devel/setup.bash"
fi
source "$px4_dir/Tools/setup_gazebo.bash" "$px4_dir" "$px4_dir/build/px4_sitl_default"
export ROS_PACKAGE_PATH="$(dirname -- "$demo_dir"):$px4_dir:$px4_dir/Tools/sitl_gazebo:${ROS_PACKAGE_PATH:-}"
exec roslaunch "$demo_dir/ugv_sim.launch" "$@"
