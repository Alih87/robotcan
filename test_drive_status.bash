#!/bin/bash

set -e

sudo ip link delete vcan0 2>/dev/null || true

sudo modprobe vcan

sudo ip link add dev vcan0 type vcan

sudo ip link set up vcan0

counter=0

send_drive_status() {
    local mode_hex="$1"
    local direction_hex="$2"
    local speed_hex="$3"
    local steering_hex="$4"
    local flags_hex="$5"
    local alarm_hex="$6"
    local spare_hex="$7"

    local counter_hex
    counter_hex=$(printf "%02X" $((counter & 0xFF)))

    local frame="${mode_hex}${direction_hex}${speed_hex}${steering_hex}${flags_hex}${alarm_hex}${spare_hex}${counter_hex}"

    echo "TX 201#$frame"
    cansend vcan0 "201#$frame"

    counter=$((counter + 1))
}

# Byte values:
# Mode:      D = 44
# Direction: F = 46, R = 52, S = 53
# Speed:     Stop = 00, Low = 01, Middle = 02, High = 03
# Steering:  00 = 0 degrees

while true; do
    echo "Forward low speed"

    for i in {1..40}; do
        send_drive_status 44 46 01 00 00 00 00
        sleep 0.05
    done

    echo "Forward middle speed"

    for i in {1..40}; do
        send_drive_status 44 46 02 00 00 00 00
        sleep 0.05
    done

    echo "Forward high speed"

    for i in {1..40}; do
        send_drive_status 44 46 03 00 00 00 00
        sleep 0.05
    done

    echo "Stop before reverse"

    for i in {1..60}; do
        send_drive_status 44 53 00 00 00 00 00
        sleep 0.05
    done

    echo "Reverse low speed"

    for i in {1..40}; do
        send_drive_status 44 52 01 00 00 00 00
        sleep 0.05
    done

    echo "Reverse middle speed"

    for i in {1..40}; do
        send_drive_status 44 52 02 00 00 00 00
        sleep 0.05
    done

    echo "Reverse high speed"

    for i in {1..40}; do
        send_drive_status 44 52 03 00 00 00 00
        sleep 0.05
    done

    echo "Final stop"

    for i in {1..80}; do
        send_drive_status 44 53 00 00 00 00 00
        sleep 0.05
    done
done