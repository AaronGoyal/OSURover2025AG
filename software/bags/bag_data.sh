# Add validation and timestamp to ensure uniqueness
if [ -z "$1" ]; then
    echo "Usage: $0 <bag_name>"
    exit 1
fi

ros2 bag record \
  -s mcap -o ~/Rover_2023_2024/software/bags/$1 \
  /rover_odom \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/aligned_depth_to_color/image_raw \
