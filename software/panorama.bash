#!/bin/bash

ros2 topic pub --once /tower/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{should_center: true, relative_pan_adjustment: 0, relative_tilt_adjustment: 0}"
sleep 0.5
ros2 topic pub --once /tower/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{should_center: false, relative_pan_adjustment: 800, relative_tilt_adjustment: 0}"
sleep 0.5

image_dir="/home/makemorerobot/images"

check_image() {
  if [ -f "$image_dir$1" ]; then
    echo "$1 saved"
  else
    echo "$1 not successfully saved"    
  fi
}

for i in {0..9}
do
  ros2 topic pub -r 30 --once /tower/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{should_center: false, relative_pan_adjustment: -160, relative_tilt_adjustment: 0}"
  sleep 1    
  ffmpeg -y -loglevel quiet -f v4l2 -input_format mjpeg -video_size 800x600 -i /dev/rover/camera_main_navigation -frames:v 1 "$image_dir/pano_$i.jpg"
  sleep 1    
  check_image "/pano_$i.jpg"
done 

read -p "Press enter when rover is positioned to scan other 180 deg..." -n 1 -s
ros2 topic pub --once /tower/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{should_center: true, relative_pan_adjustment: 0, relative_tilt_adjustment: 0}"
sleep 0.5
ros2 topic pub --once /tower/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{should_center: false, relative_pan_adjustment: 800, relative_tilt_adjustment: 0}"
sleep 0.5


for i in {10..19}
do
  ros2 topic pub -r 30 --once /tower/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{should_center: false, relative_pan_adjustment: -160, relative_tilt_adjustment: 0}"
  sleep 1
  ffmpeg -y -loglevel quiet -f v4l2 -input_format mjpeg -video_size 800x600 -i /dev/rover/camera_main_navigation -frames:v 1 "$image_dir/pano_$i.jpg"
  sleep 1
  
  check_image "/pano_$i.jpg"

done 

scp "$image_dir" makemorerobot@192.168.1.1:/home

