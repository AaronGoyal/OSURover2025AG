#!/bin/bash
gst-launch-1.0 v4l2src device=/dev/rover/camera_chassis !   video/x-raw,width=640,height=480,framerate=30/1 !   nvvidconv ! nvv4l2h265enc !   h265parse !   rtph265pay config-interval=1 !   udpsink host=192.168.1.1 port=42067 sync=false &
gst-launch-1.0 v4l2src device=/dev/rover/camera_infrared !   video/x-raw,width=640,height=480,framerate=30/1 !   nvvidconv !   nvv4l2h265enc !   udpsink host=192.168.1.1 port=42068 sync=false &
gst-launch-1.0 v4l2src device=/dev/rover/camera_main_navigation !   video/x-raw,width=640,height=480,framerate=30/1 !   nvvidconv !  nvv4l2h265enc !   h265parse !   rtph265pay config-interval=1 !   udpsink host=192.168.1.1 port=42069 sync=false &
wait
