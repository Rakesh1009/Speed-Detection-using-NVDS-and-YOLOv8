#!/bin/bash
# set -e  # Exit on first error
make clean
make -j$(nproc)
# export GST_DEBUG=3  # Set to the desired level (1-9)
./ds-yolov8
# ./get_data.sh
# gst-launch-1.0 v4l2src ! videoconvert ! autovideosink
# gst-launch-1.0 v4l2src device=/dev/video0 ! image/jpeg,width=2560,height=1920,framerate=30/1 ! jpegdec ! videoconvert ! xvimagesink
# gst-inspect-1.0 jpegdec