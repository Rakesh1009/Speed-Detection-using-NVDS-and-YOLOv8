#!/bin/bash
set -e  # Exit on first error
make clean
make -j$(nproc)
./ds-yolov8