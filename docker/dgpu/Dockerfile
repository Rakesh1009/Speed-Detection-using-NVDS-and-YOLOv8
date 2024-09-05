FROM nvcr.io/nvidia/deepstream:7.0-gc-triton-devel
# Add open GL libraries
RUN apt-get update && apt-get install -y libopencv-dev
RUN apt-get update && \
        DEBIAN_FRONTEND=noninteractive  apt-get install -y --no-install-recommends \
        pkg-config \
        libgnutls28-dev \
        libboost-all-dev
# RUN git clone https://github.com/mpromonet/v4l2rtspserver.git /opt/v4l2rtspserver \
#         && cd /opt/v4l2rtspserver \
#         && mkdir build && cd build \
#         && cmake .. && make
RUN rm -rf /var/lib/apt/lists/* && \
        apt autoremove
RUN export DISPLAY=:1
# RUN xhost +
WORKDIR /app
# To get video driver libraries at runtime (libnvidia-encode.so/libnvcuvid.so)
ENV NVIDIA_DRIVER_CAPABILITIES=$NVIDIA_DRIVER_CAPABILITIES,video,compute,graphics,utility
ENV CUDA_HOME=/usr/local/cuda
ENV CFLAGS="-I$CUDA_HOME/include $CFLAGS"