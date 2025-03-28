sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

sudo ip link set can1 down
sudo ip link set can1 up type can bitrate 500000

sudo modprobe v4l2loopback devices=8 video_nr=10,11,12,13,14,15,16,17 card_label="virtual"
gst-launch-1.0 v4l2src device=/dev/rover/camera_infrared ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! v4l2sink device=/dev/video10 &
gst-launch-1.0 v4l2src device=/dev/rover/camera_chassis ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! v4l2sink device=/dev/video11 &
gst-launch-1.0 v4l2src device=/dev/rover/camera_main_navigation ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! v4l2sink device=/dev/video12 &
gst-launch-1.0 rtspsrc location=rtsp://192.168.1.11:554 ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! v4l2sink device=/dev/video13 &

source /home/makemorerobot/Rover_2023_2024/software/install/setup.bash
ros2 launch rover2_main rover2_main_launch.py
