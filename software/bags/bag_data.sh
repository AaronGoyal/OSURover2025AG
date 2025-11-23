# Add validation and timestamp to ensure uniqueness
if [ -z "$1" ]; then
    echo "Usage: $0 <bag_name>"
    exit 1
fi

ros2 bag record \
  -s mcap -o ~/Rover_2023_2024/software/bags/$1 \
  /tf_static \
  /odometry/global \
  /odom \
  /imu/data \
  /gps/fix \
  /camera/d455/color/image_raw \
  /camera/d455/color/camera_info \
  /camera/d455/aligned_depth_to_color/image_raw \
  /tf
