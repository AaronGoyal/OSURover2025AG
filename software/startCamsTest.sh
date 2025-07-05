
#!/bin/bash
#echo "Starting loopback..."
sudo modprobe v4l2loopback devices=6 video_nr=20,21,22,23,24,25 card_label="virtual", max_buffers=2
sleep 5
echo "Starting camera_infrared loopback"
gst-launch-1.0 v4l2src device=/dev/rover/camera_infrared ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegdec ! videoconvert ! v4l2sink device=/dev/video20 &
sleep 1
echo "Starting camera_chassis loopback"
#gst-launch-1.0 v4l2src device=/dev/rover/camera_chassis ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegdec ! videoconvert ! v4l2sink device=/dev/video21 &
#sleep 1
echo "Starting camera_main_navigation loopback"
gst-launch-1.0 v4l2src device=/dev/rover/camera_main_navigation ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegdec ! videoconvert ! v4l2sink device=/dev/video22 &
sleep 1
#echo "Starting camera_chassis_455_ir loopback"
#gst-launch-1.0 v4l2src device=/dev/rover/camera_chassis_455_ir ! videoflip method=rotate-180 ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! v4l2sink device=/dev/video23 &
#sleep 1
#echo "Starting camera_chassis_455_rgb loopback"
#gst-launch-1.0 v4l2src device=/dev/rover/camera_chassis_455_rgb ! videoflip method=rotate-180 ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! v4l2sink device=/dev/video24 &
#sleep 1

systemd-notify --ready


#echo "Starting gripper IP cam loopback"
#Wait for camera to be actually reachable
#while ! nc -zv 192.168.1.11 554 >/dev/null 2>&1; do
#    echo "Waiting for camera at 192.168.1.11:554..."
#    sleep 10
#done
#echo "Camera accessible, starting ffmpeg..."
#ffmpeg -rtsp_transport tcp -i rtsp://192.168.1.11:554 -vf format=yuv420p -f v4l2 /dev/video25 &
#gst-launch-1.0 rtspsrc location=rtsp://192.168.1.11:554 ! image/jpeg,width=352,height=288,framerate=25/1 ! jpegdec ! videoconvert ! v4l2sink device=/dev/video25 &


wait

