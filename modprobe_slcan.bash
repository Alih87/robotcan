#!/bin/bash
sudo slcand -o -c -s6 -S 115200 /dev/ttyUSB0 can0
sudo ip link set can0 up
