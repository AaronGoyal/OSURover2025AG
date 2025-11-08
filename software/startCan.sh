#!/bin/bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

sudo ip link set can0 down
sleep 1
sudo ip link set can1 down
sleep 1
sudo ip link set can1 txqueuelen 150
sleep 1
sudo ip link set can1 up type can bitrate 1000000
sleep 1
sudo ip link set can0 up type can bitrate 500000
sleep 1
echo "Can net started"
