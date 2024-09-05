#!/bin/bash
set -e  # Exit on first error
make clean
make -j$(nproc)
# export GST_DEBUG=3  # Set to the desired level (1-9)
./ds-yolov8

# gst-launch-1.0 v4l2src ! videoconvert ! autovideosink
