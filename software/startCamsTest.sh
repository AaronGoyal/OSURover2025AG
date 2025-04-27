#!/bin/bash
sudo modprobe v4l2loopback devices=8 video_nr=10,11,12,13 card_label="virtual", max_buffers=2
sleep 2
gst-launch-1.0 v4l2src device=/dev/rover/camera_infrared ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegdec ! videoconvert ! v4l2sink device=/dev/video10 &
sleep 1
gst-launch-1.0 v4l2src device=/dev/rover/camera_chassis ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegdec ! videoconvert ! v4l2sink device=/dev/video11 &
sleep 1
gst-launch-1.0 v4l2src device=/dev/rover/camera_main_navigation ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegdec ! videoconvert ! v4l2sink device=/dev/video12 &
sleep 1
gst-launch-1.0 rtspsrc location=rtsp://192.168.1.11:554 ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! v4l2sink device=/dev/video13 &

systemd-notify --ready

wait

