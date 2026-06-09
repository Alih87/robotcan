#!/bin/bash
sudo ip link delete vcan0
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

while true; do
	cansend vcan0 102#DEADBEEF
	sleep 0.05
done
