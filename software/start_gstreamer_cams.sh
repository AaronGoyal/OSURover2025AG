#!/bin/bash
echo "Starting streams..."
sleep 1
echo "Starting camera_infrared stream"
gst-launch-1.0 v4l2src device=/dev/video20 !  videoconvert !  video/x-raw,width=640,height=480,framerate=30/1 !   nvvidconv ! nvv4l2h265enc !   h265parse !   rtph265pay config-interval=1 !   udpsink host=192.168.1.1 port=42067 sync=false &
sleep 1
echo "Starting camera_chassis stream"
gst-launch-1.0 v4l2src device=/dev/video21 !  videoconvert !  video/x-raw,width=640,height=480,framerate=30/1 !   nvvidconv ! nvv4l2h265enc !   h265parse !   rtph265pay config-interval=1 !   udpsink host=192.168.1.1 port=42068 sync=false &
sleep 1
echo "Starting camera_main_navigation stream"
gst-launch-1.0 v4l2src device=/dev/video22 !  videoconvert !  video/x-raw,width=640,height=480,framerate=30/1 !   nvvidconv ! nvv4l2h265enc !   h265parse !   rtph265pay config-interval=1 !   udpsink host=192.168.1.1 port=42069 sync=false &
sleep 1
echo "Starting gripper_ip stream"
gst-launch-1.0 v4l2src device=/dev/video25 !  videoconvert !  video/x-raw,width=352,height=288,framerate=30/1 !   nvvidconv ! nvv4l2h265enc !   h265parse !   rtph265pay config-interval=1 !   udpsink host=192.168.1.1 port=42070 sync=false &
wait
