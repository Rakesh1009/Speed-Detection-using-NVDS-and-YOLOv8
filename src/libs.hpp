#include <gst/gst.h>
#include <glib.h>

#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>

#include "gstnvdsmeta.h"
#include "nvdsmeta_schema.h"
#include "nvbufsurface.h"

#ifndef PLATFORM_TEGRA
#include "gst-nvmessage.h"
#endif

#include <cuda.h>

#include <string>
#include <fstream>
#include <cmath>

#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace std::chrono;

#define SOURCE_PATH "inputsources.txt"


// Define format identifiers
#define FORMAT_MJPEG 1
#define FORMAT_YUYV  2

// Set the input format using the integer identifiers
#define INPUTFORMAT 2

// Preprocessor logic to set width, height, and FPS based on format
#if INPUTFORMAT == FORMAT_MJPEG
    #define INPUTWIDTH  2560
    #define INPUTHEIGHT 1920
    #define INPUTFPS    30
    #define MUXER_OUTPUT_WIDTH 2560
    #define MUXER_OUTPUT_HEIGHT 1920
    #define TILED_OUTPUT_WIDTH 2560
    #define TILED_OUTPUT_HEIGHT 1920
    #define INPUTFORMAT_STR "image/jpeg"
#elif INPUTFORMAT == FORMAT_YUYV
    #define INPUTWIDTH  640
    #define INPUTHEIGHT 480
    #define INPUTFPS    30
    #define MUXER_OUTPUT_WIDTH 640
    #define MUXER_OUTPUT_HEIGHT 480
    #define TILED_OUTPUT_WIDTH 640
    #define TILED_OUTPUT_HEIGHT 480
    #define INPUTFORMAT_STR "video/x-raw"
#else
    #error "Unsupported INPUTFORMAT"
#endif

#define MULTIPLIER TILED_OUTPUT_HEIGHT/480.0

//length of trail
#define TRAIL_LEN 40

//number of speeds 
#define NUM_SPEED 30

//number of tracks
#define NUM_TRACKS 15

#define PERF_INTERVAL 2

#define MAX_DISPLAY_LEN 64

// Network Compute Mode
#define COMPUTE_MODE "fp16"

#define MAX_TRACKING_ID_LEN 16

/* Muxer batch formation timeout, for e.g. 40 millisec. Should ideally be set
 * based on the fastest source's framerate. */
#define MUXER_BATCH_TIMEOUT_USEC 4000000

/* NVIDIA Decoder source pad memory feature. This feature signifies that source
 * pads having this capability will push GstBuffers containing cuda buffers. */
#define GST_CAPS_FEATURES_NVMM "memory:NVMM"

#define CHECK_ERROR(error)                                               \
  if (error)                                                             \
  {                                                                      \
    g_printerr("Error while parsing config file: %s\n", error->message); \
    goto done;                                                           \
  }

#define CONFIG_GROUP_TRACKER "tracker"
#define CONFIG_GROUP_TRACKER_WIDTH "tracker-width"
#define CONFIG_GROUP_TRACKER_HEIGHT "tracker-height"
#define CONFIG_GROUP_TRACKER_LL_CONFIG_FILE "ll-config-file"
#define CONFIG_GROUP_TRACKER_LL_LIB_FILE "ll-lib-file"
#define CONFIG_GROUP_TRACKER_ENABLE_BATCH_PROCESS "enable-batch-process"
#define CONFIG_GPU_ID "gpu-id"

int num_sources = 0;
