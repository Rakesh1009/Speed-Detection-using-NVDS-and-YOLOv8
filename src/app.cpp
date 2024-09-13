#include "app.hpp"

namespace YOLOv8
{

  gboolean YOLOv8::Odin::display_off = FALSE;
  float YOLOv8::Odin::distance = 0.0f;

  CircularBuffer<ObjectData> YOLOv8::Odin::objectdatamap(NUM_TRACKS);

  //updating fps
  void
  Odin::update_fps(gint id)
  {

    gdouble current_fps = duration_cast<std::chrono::milliseconds>(system_clock::now() - fps[id].fps_timer).count();
    current_fps /= 1000;
    current_fps = 1 / current_fps;
    fps[id].rolling_fps = (gint)(fps[id].rolling_fps * 0.7 + current_fps * 0.3);
    auto timer = duration_cast<std::chrono::seconds>(system_clock::now() - fps[id].display_timer).count();
    if (timer > PERF_INTERVAL)
    {
      fps[id].display_fps = fps[id].rolling_fps;
      fps[id].display_timer = system_clock::now();
    }
    fps[id].fps_timer = system_clock::now();
  }

  //connecting the input sources to the streammux
  int Odin::create_input_sources(gpointer pipe, gpointer mux, guint num_sources)
  {

    GstElement *pipeline = (GstElement *)pipe;
    GstElement *streammux = (GstElement *)mux;

    std::ifstream infile(SOURCE_PATH);
    std::string source;

    if (infile.is_open())
    {
      while (getline(infile, source))
      {

        if (source.empty() || source[0] == '#')
        {
          continue; // Skip this iteration if the line is a comment
        }

        GstPad *sinkpad, *srcpad;
        GstElement *source_bin = NULL;
        gchar pad_name[16] = {};

        source_bin = create_source_bin(num_sources, (gchar *)source.c_str());

        if (!source_bin)
        {
          g_printerr("Failed to create source bin. Exiting.\n");
          return -1;
        }

        gst_bin_add(GST_BIN(pipeline), source_bin);

        g_snprintf(pad_name, 15, "sink_%u", num_sources);
        sinkpad = gst_element_request_pad_simple(streammux, pad_name);
        if (!sinkpad)
        {
          g_printerr("Streammux request sink pad failed. Exiting.\n");
          return -1;
        }

        srcpad = gst_element_get_static_pad(source_bin, "src");
        if (!srcpad)
        {
          g_printerr("Failed to get src pad of source bin. Exiting.\n");
          return -1;
        }

        if (gst_pad_link(srcpad, sinkpad) != GST_PAD_LINK_OK)
        {
          g_printerr("Failed to link source bin to stream muxer. Exiting.\n");
          return -1;
        }

        gst_object_unref(srcpad);
        gst_object_unref(sinkpad);
        num_sources++;
      }
    }
    infile.close();
    return num_sources;
  }

  //chaning the features of the bounding box
  void
  Odin::changeBBoxColor(gpointer obj_meta_data, int has_bg_color, float red, float green, float blue, float alpha)
  {

    NvDsObjectMeta *obj_meta = (NvDsObjectMeta *)obj_meta_data;
  #ifndef PLATFORM_TEGRA
      obj_meta->rect_params.has_bg_color = has_bg_color;
      obj_meta->rect_params.bg_color.red = red;
      obj_meta->rect_params.bg_color.green = green;
      obj_meta->rect_params.bg_color.blue = blue;
      obj_meta->rect_params.bg_color.alpha = alpha;
  #endif
      obj_meta->rect_params.border_color.red = red;
      obj_meta->rect_params.border_color.green = green;
      obj_meta->rect_params.border_color.blue = blue;
      obj_meta->rect_params.border_color.alpha = alpha;
      obj_meta->text_params.font_params.font_size = 8 * MULTIPLIER;
  }

  //adding display meta data about objects on screen like text boxes, lines etc
  void
  Odin::addDisplayMeta(gpointer batch_meta_data, gpointer frame_meta_data)
  {

    NvDsBatchMeta *batch_meta = (NvDsBatchMeta *)batch_meta_data;
    NvDsFrameMeta *frame_meta = (NvDsFrameMeta *)frame_meta_data;

    NvDsDisplayMeta *display_meta = nvds_acquire_display_meta_from_pool(batch_meta);
    NvOSD_TextParams *txt_params = NULL;
    NvOSD_LineParams *line_params = NULL;
    int offset = 0;

    // Initialize display_meta
    display_meta->num_labels = 0;  // Reset label count
    display_meta->num_lines = 0;   // Reset line count

    // Add FPS information
    txt_params = &display_meta->text_params[display_meta->num_labels];
    txt_params->display_text = (char *)g_malloc0(MAX_DISPLAY_LEN);

    update_fps(frame_meta->source_id);

    offset = snprintf(txt_params->display_text, MAX_DISPLAY_LEN, "Source: %d | FPS: %d",
                      frame_meta->source_id, fps[frame_meta->source_id].display_fps);

    txt_params->x_offset = 10*MULTIPLIER;
    txt_params->y_offset = 12*MULTIPLIER;

    txt_params->font_params.font_name = (char *)"Serif";
    txt_params->font_params.font_size = 8 * MULTIPLIER;
    txt_params->font_params.font_color.red = 1.0;
    txt_params->font_params.font_color.green = 1.0;
    txt_params->font_params.font_color.blue = 1.0;
    txt_params->font_params.font_color.alpha = 1.0;

    txt_params->set_bg_clr = 1;
    txt_params->text_bg_clr.red = 0.0;
    txt_params->text_bg_clr.green = 0.0;
    txt_params->text_bg_clr.blue = 0.0;
    txt_params->text_bg_clr.alpha = 1.0;

    display_meta->num_labels++;

    //  //draw 2 liens vertical and horizxontal passing through center

      // NvOSD_LineParams *line_params_temp1 = &display_meta->line_params[display_meta->num_lines]; 

      // //vertical  
      // line_params_temp1->x1 = TILED_OUTPUT_WIDTH/2;
      // line_params_temp1->y1 = 0;
      // line_params_temp1->x2 = TILED_OUTPUT_WIDTH/2;
      // line_params_temp1->y2 = TILED_OUTPUT_HEIGHT;

      // line_params_temp1->line_width = 2*MULTIPLIER; // Example: Set line width to 2 pixels
      // line_params_temp1->line_color.red = 0.0;
      // line_params_temp1->line_color.green = 0.0;
      // line_params_temp1->line_color.blue = 0.0;
      // line_params_temp1->line_color.alpha = 1.0;
      
      // display_meta->num_lines++;

      // NvOSD_LineParams *line_params_temp2 = &display_meta->line_params[display_meta->num_lines];

      // //horizontal
      // line_params_temp2->x1 = 0;
      // line_params_temp2->y1 = TILED_OUTPUT_HEIGHT/2;
      // line_params_temp2->x2 = TILED_OUTPUT_WIDTH;
      // line_params_temp2->y2 = TILED_OUTPUT_HEIGHT/2;

      // line_params_temp2->line_width = 2*MULTIPLIER; // Example: Set line width to 2 pixels
      // line_params_temp2->line_color.red = 0.0;
      // line_params_temp2->line_color.green = 0.0;
      // line_params_temp2->line_color.blue = 0.0;
      // line_params_temp2->line_color.alpha = 1.0;

      // display_meta->num_lines++;

    double speed = 0.0;
    float max_speed = 2.0;
    float red = 0.0;
    float green = 0.0;
    float blue = 0.0;

    for (NvDsMetaList *l_obj = frame_meta->obj_meta_list; l_obj != NULL; l_obj = l_obj->next)
    {
      NvDsObjectMeta *obj_meta = (NvDsObjectMeta *)(l_obj->data);
      if (!obj_meta)
        continue;
      
      snprintf(obj_meta->text_params.display_text, MAX_DISPLAY_LEN, "");

      speed = calculate_object_speed(obj_meta);
      
      // g_print("reaching here\n");
      if (speed > max_speed)
      {
        red = 1.0;
        green = 0.0;
      }

      if (speed < 0.0)
      {
        red = 0.0;
        green = 1.0;
      }

      if (speed <= max_speed && speed >= 0.0)
      {
        red = speed / max_speed;
        green = 1.0 - red;
      }

      changeBBoxColor(obj_meta, 1, red, green, blue, 0.25);
     
      vector<float> x_points = objectdatamap[obj_meta->object_id % NUM_TRACKS].prev_x.getValues();
      vector<float> y_points = objectdatamap[obj_meta->object_id % NUM_TRACKS].prev_y.getValues();

      for (size_t i = 0; i < x_points.size() - 1; i++)
      {
        NvOSD_LineParams *line_params = &display_meta->line_params[display_meta->num_lines];

        // Set the start and end points of the line
        line_params->x1 = x_points[i];
        line_params->y1 = y_points[i];
        line_params->x2 = x_points[i + 1];
        line_params->y2 = y_points[i + 1];
        float min_ = 0.0;
        // Set the line color and width
        line_params->line_width = 4*MULTIPLIER; // Example: Set line width to 2 pixels
        line_params->line_color.red = red;
        line_params->line_color.green = green;
        line_params->line_color.blue = blue;
        line_params->line_color.alpha = min_ + ((1 - min_) * i) / (x_points.size() - 1);

        // Increment the line count
        display_meta->num_lines++;
      }
    }

      txt_params = &display_meta->text_params[display_meta->num_labels];
      txt_params->display_text = (char *)g_malloc0(MAX_DISPLAY_LEN);

      // Format the speed information
      snprintf(txt_params->display_text, MAX_DISPLAY_LEN, "Speed: %.2f m/s", speed);

      txt_params->x_offset = TILED_OUTPUT_WIDTH - 150*MULTIPLIER;
      txt_params->y_offset = 3*12*MULTIPLIER;

      // Set font, color, and background properties
      txt_params->font_params.font_name = (char *)"Serif";
      txt_params->font_params.font_size = 8 * MULTIPLIER;
      txt_params->font_params.font_color.red = 1.0;
      txt_params->font_params.font_color.green = 1.0;
      txt_params->font_params.font_color.blue = 1.0;
      txt_params->font_params.font_color.alpha = 1.0;

      txt_params->set_bg_clr = 1;
      txt_params->text_bg_clr.red = 0.0;
      txt_params->text_bg_clr.green = 0.0;
      txt_params->text_bg_clr.blue = 0.0;
      txt_params->text_bg_clr.alpha = 1.0;

      // Increment the label count
      display_meta->num_labels++;


    txt_params = &display_meta->text_params[display_meta->num_labels];
    txt_params->display_text = (char *)g_malloc0(MAX_DISPLAY_LEN);

    offset = snprintf(txt_params->display_text, MAX_DISPLAY_LEN, "Distance: %.2f m", distance);

    txt_params->x_offset = TILED_OUTPUT_WIDTH - 150*MULTIPLIER;
    txt_params->y_offset = 12*MULTIPLIER;

    txt_params->font_params.font_name = (char *)"Serif";
    txt_params->font_params.font_size = 8 * MULTIPLIER;
    txt_params->font_params.font_color.red = 1.0;
    txt_params->font_params.font_color.green = 1.0;
    txt_params->font_params.font_color.blue = 1.0;
    txt_params->font_params.font_color.alpha = 1.0;

    txt_params->set_bg_clr = 1;
    txt_params->text_bg_clr.red = 0.0;
    txt_params->text_bg_clr.green = 0.0;
    txt_params->text_bg_clr.blue = 0.0;
    txt_params->text_bg_clr.alpha = 1.0;

    display_meta->num_labels++;

    txt_params = &display_meta->text_params[display_meta->num_labels];
    txt_params->display_text = (char *)g_malloc0(MAX_DISPLAY_LEN);

    // std::string s;

    int num_l = 0;
    int total_length = 100;

    speed = float(speed);

    num_l = (speed * total_length) / max_speed;

   if(num_l > total_length) {
     num_l = total_length;
   }

    // g_print("num_l: %d\n", num_l);

    std::string s(num_l, '|'); 
    s += std::string(total_length - num_l, ' ');

    offset = snprintf(txt_params->display_text, MAX_DISPLAY_LEN, s.c_str());

    txt_params->x_offset = 10*MULTIPLIER;
    txt_params->y_offset = 3*12*MULTIPLIER;

    txt_params->font_params.font_name = (char *)"Serif";
    txt_params->font_params.font_size = 8 * MULTIPLIER;
    txt_params->font_params.font_color.red = red;
    txt_params->font_params.font_color.green = green;
    txt_params->font_params.font_color.blue = blue;
    txt_params->font_params.font_color.alpha = 1.0;

    txt_params->set_bg_clr = 1;
    txt_params->text_bg_clr.red = 0.0;
    txt_params->text_bg_clr.green = 0.0;
    txt_params->text_bg_clr.blue = 0.0;
    txt_params->text_bg_clr.alpha = 0.7;

    display_meta->num_labels++;

    nvds_add_display_meta_to_frame(frame_meta, display_meta);
    
  }

  
  //convert the pixel coordinates to world coordinates using some calibration parameters and assumptions like angle of camera to floor is known and distance of camera from the floor also is known (this is along the optical axis) 
  pair<float, float> Odin::convert_coordinates(float u, float v){
    float theta = atan((2*v/(TILED_OUTPUT_WIDTH*1.0))*tan(FOV_X));
    float beta = atan((2*u/(TILED_OUTPUT_HEIGHT*1.0))*tan(FOV_Y));
    float rho = asin(cos(ALPHA_Y)*cos(theta));
    float l = L*sin(beta)/(cos(theta)*sin(rho-beta));
    return make_pair(L*tan(theta)+l*sin(theta), l*cos(theta));
  }

  //calculation of detection speed, using the previous and current coordinates
  double
  Odin::calculate_object_speed(NvDsObjectMeta *obj_meta)
  {
    int object_id = obj_meta->object_id;
    auto now = std::chrono::system_clock::now();
    ObjectData &data = objectdatamap[object_id % NUM_TRACKS];

    // Calculate time elapsed
    // std::chrono::duration<double> elapsed_seconds = now - data.prev_time;
    std::chrono::duration<double> elapsed_seconds_t = now - data.last_t;

    // If first frame, initialize previous coordinates and time
    if (data.first_frame) {
        data.prev_x.addValue(obj_meta->rect_params.left + obj_meta->rect_params.width / 2.0);
        data.prev_y.addValue(obj_meta->rect_params.top + obj_meta->rect_params.height / 2.0);
        // data.prev_time = now;
        data.last_x = (obj_meta->rect_params.left + obj_meta->rect_params.width / 2.0);
        data.last_y = (obj_meta->rect_params.top + obj_meta->rect_params.height / 2.0);
        data.first_frame = false;
        data.last_d = 0.0;
        data.last_t = now;
        return 0.0;  // No distance can be calculated from a single frame
    }

    // Current coordinates
    double current_x = obj_meta->rect_params.left + obj_meta->rect_params.width / 2.0;
    double current_y = obj_meta->rect_params.top + obj_meta->rect_params.height / 2.0;

    // g_print("Last X: %f, Last Y: %f\n", data.last_x, data.last_y);
    // g_print("Current X: %f, Current Y: %f\n", current_x, current_y);
    float x_old = data.last_x-TILED_OUTPUT_WIDTH/2.0;
    float y_old = -data.last_y+TILED_OUTPUT_HEIGHT/2.0;

    float x_new = current_x-TILED_OUTPUT_WIDTH/2.0;
    float y_new = -current_y+TILED_OUTPUT_HEIGHT/2.0;
    
    pair<float, float> real_last = convert_coordinates(x_old, y_old);
    pair<float, float> real_current = convert_coordinates(x_new, y_new);
    
    float loc = 0;
    if(elapsed_seconds_t.count() > 0.01){

      data.last_d = sqrt(pow(real_current.first - real_last.first, 2) + pow(real_current.second - real_last.second, 2));
      data.last_x = current_x;
      data.last_y = current_y;
      data.last_t = now;
    }

    loc = sqrt(pow(real_current.first, 2) + pow(real_current.second, 2));

    data.prev_x.addValue(current_x);
    data.prev_y.addValue(current_y);
    // data.prev_time = now;
    data.speeds.addValue(data.last_d/elapsed_seconds_t.count());

    distance += data.last_d;

    return data.speeds.getAverage();
  }

  //tiler probe function that is being used to handled the metadata before image reaches the sink
  GstPadProbeReturn
  Odin::tiler_src_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info, gpointer u_data)
  {
    GstBuffer *buf = (GstBuffer *)info->data;

    // To access the entire batch data
    NvDsBatchMeta *batch_meta = NULL;

    NvDsObjectMeta *obj_meta = NULL;
    NvDsFrameMeta *frame_meta = NULL;

    // To access the frame
    NvBufSurface *surface = NULL;
    // TO generate message meta
    NvDsEventMsgMeta *msg_meta = NULL;

    NvDsMetaList *l_frame = NULL;
    NvDsMetaList *l_obj = NULL;

    // Get original raw data
    GstMapInfo in_map_info;
    // char *src_data = NULL;

    if (!gst_buffer_map(buf, &in_map_info, GST_MAP_READ))
    {
      g_print("Error: Failed to map gst buffer\n");
      gst_buffer_unmap(buf, &in_map_info);
      return GST_PAD_PROBE_OK;
    }


    GstMapInfo inmap = GST_MAP_INFO_INIT;
    if (!gst_buffer_map(buf, &inmap, GST_MAP_READ)) {
        std::cerr << "Error: input buffer mapinfo failed" << std::endl;
        return GST_PAD_PROBE_OK;
    }

    batch_meta = gst_buffer_get_nvds_batch_meta(buf);

    if (!batch_meta)
    {
      return GST_PAD_PROBE_OK;
    }

    for (l_frame = batch_meta->frame_meta_list; l_frame != NULL; l_frame = l_frame->next)
    {

      frame_meta = (NvDsFrameMeta *)(l_frame->data);

      if (frame_meta == NULL)
        continue;
      addDisplayMeta(batch_meta, frame_meta);
    }

    gst_buffer_unmap(buf, &in_map_info);
    return GST_PAD_PROBE_OK;
  }

  //the bus call function that waits for error messages, warning messages and EOS
  gboolean
  Odin::bus_call(GstBus *bus, GstMessage *msg, gpointer data)
  {
    GMainLoop *loop = (GMainLoop *)data;
    switch (GST_MESSAGE_TYPE(msg))
    {
    case GST_MESSAGE_EOS:
      g_print("End of stream\n");
      g_main_loop_quit(loop);
      break;
    case GST_MESSAGE_WARNING:
    {
      gchar *debug;
      GError *error;
      gst_message_parse_warning(msg, &error, &debug);
      g_printerr("WARNING from element %s: %s\n",
                 GST_OBJECT_NAME(msg->src), error->message);
      g_free(debug);
      g_printerr("Warning: %s\n", error->message);
      g_error_free(error);
      break;
    }
    case GST_MESSAGE_ERROR:
    {
      gchar *debug;
      GError *error;
      gst_message_parse_error(msg, &error, &debug);
      g_printerr("ERROR from element %s: %s\n",
                 GST_OBJECT_NAME(msg->src), error->message);
      if (debug)
        g_printerr("Error details: %s\n", debug);
      g_free(debug);
      g_error_free(error);
      g_main_loop_quit(loop);
      break;
    }
#ifndef PLATFORM_TEGRA
    case GST_MESSAGE_ELEMENT:
    {
      if (gst_nvmessage_is_stream_eos(msg))
      {
        guint stream_id;
        if (gst_nvmessage_parse_stream_eos(msg, &stream_id))
        {
          g_print("Got EOS from stream %d\n", stream_id);
        }
      }
      break;
    }
#endif
    default:
      break;
    }
    return TRUE;
  }

  //callback function that is called when a new source pad is added
  void
  Odin::cb_newpad(GstElement *decodebin, GstPad *decoder_src_pad, gpointer data)
  {
    g_print("In cb_newpad\n");
    GstCaps *caps = gst_pad_get_current_caps(decoder_src_pad);
    const GstStructure *str = gst_caps_get_structure(caps, 0);
    const gchar *name = gst_structure_get_name(str);
    GstElement *source_bin = (GstElement *)data;
    GstCapsFeatures *features = gst_caps_get_features(caps, 0);

    /* Need to check if the pad created by the decodebin is for video and not
     * audio. */
    if (!strncmp(name, "video", 5))
    {
      /* Link the decodebin pad only if decodebin has picked nvidia
       * decoder plugin nvdec_*. We do this by checking if the pad caps contain
       * NVMM memory features. */
      if (gst_caps_features_contains(features, GST_CAPS_FEATURES_NVMM))
      {
        /* Get the source bin ghost pad */
        GstPad *bin_ghost_pad = gst_element_get_static_pad(source_bin, "src");
        if (!gst_ghost_pad_set_target(GST_GHOST_PAD(bin_ghost_pad),
                                      decoder_src_pad))
        {
          g_printerr("Failed to link decoder src pad to source bin ghost pad\n");
        }
        gst_object_unref(bin_ghost_pad);
      }
      else
      {
        g_printerr("Error: Decodebin did not pick nvidia decoder plugin.\n");
      }
    }
  }

  //callback function that is called when a new child is added to the bin, this is used recursively
  void
  Odin::decodebin_child_added(GstChildProxy *child_proxy, GObject *object, gchar *name, gpointer user_data)
  {
    g_print("Decodebin child added: %s\n", name);
    if (g_strrstr(name, "decodebin") == name)
    {
      g_signal_connect(G_OBJECT(object), "child-added",
                       G_CALLBACK(decodebin_child_added), user_data);
    }
    if (g_strstr_len(name, -1, "nvv4l2decoder") == name)
    {
      g_print("Setting bufapi_version\n");
      g_object_set(object, "bufapi-version", TRUE, NULL);
    }
  }
  
  //creating input sources, depending on whether the sources are file type or rtsp type or v4l2src type(webcam) input bin creation was handled
  GstElement *
  Odin::create_source_bin(guint index, gchar *uri)
  {
    GstElement *bin = NULL, *source_element = NULL, *cap_filter1 = NULL, *cap_filter = NULL, *nvvidconv2 = NULL, *nvvidconv1 = NULL, *jpgdec = NULL;
    GstCaps *caps = NULL, *caps1 = NULL, *convertcaps = NULL;
    GstCapsFeatures *feature = NULL;
    // GstCaps
    gchar bin_name[16] = {};

    g_snprintf(bin_name, sizeof(bin_name), "source-bin-%02d", index);
    bin = gst_bin_new(bin_name);

    if (!bin)
    {
      g_printerr("Failed to create source bin.\n");
      return NULL;
    }

    if (g_str_has_prefix(uri, "camera://"))
    {

      std::string devicePath = uri + 9; // Remove "camera://"
      g_print("Camera device: %s\n", devicePath.c_str());
      source_element = gst_element_factory_make("v4l2src", "camera-src");
      if (!source_element)
      {
        g_printerr("Failed to create camera elements.\n");
        gst_object_unref(bin); // Unref bin to avoid memory leak
        return NULL;
      }

      g_object_set(G_OBJECT(source_element), "device", devicePath.c_str(), NULL);
      cap_filter1 = gst_element_factory_make("capsfilter", "v4l2src_caps");
      if (!cap_filter1)
      {
        g_printerr("Failed to create camera elements.\n");
      }
      caps1 = gst_caps_new_simple(INPUTFORMAT_STR,
                                  "width", G_TYPE_INT, INPUTWIDTH, "height", G_TYPE_INT,
                                  INPUTHEIGHT, "framerate", GST_TYPE_FRACTION,
                                  INPUTFPS, 1, NULL);

      cap_filter = gst_element_factory_make("capsfilter", "caps_filter");

      if (!cap_filter)
      {
        g_printerr("Failed to create camera elements.\n");
        gst_object_unref(bin); // Unref bin to avoid memory leak
        return NULL;
      }

      if(INPUTFORMAT_STR == "image/jpeg") jpgdec = gst_element_factory_make("jpegdec", "jpegdec1");

      caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING,
                                 "NV12", "width", G_TYPE_INT, INPUTWIDTH,
                                 "height", G_TYPE_INT, INPUTHEIGHT, "framerate",
                                 GST_TYPE_FRACTION, INPUTFPS, 1,
                                 NULL);

      nvvidconv1 = gst_element_factory_make("videoconvert", "nvvidconv1");
      feature = gst_caps_features_new("memory:NVMM", NULL);
      gst_caps_set_features(caps, 0, feature);

      g_object_set(G_OBJECT(cap_filter), "caps", caps, NULL);

      g_object_set(G_OBJECT(cap_filter1), "caps", caps1, NULL);

      nvvidconv2 = gst_element_factory_make("nvvideoconvert", "nvvidconv2");

      g_object_set(G_OBJECT(nvvidconv2), "gpu-id", 0,
                   "nvbuf-memory-type", 0, NULL);
      if(INPUTFORMAT_STR == "image/jpeg") gst_bin_add_many(GST_BIN(bin), source_element, cap_filter1, jpgdec,
                       nvvidconv1, nvvidconv2, cap_filter, NULL);
      else gst_bin_add_many(GST_BIN(bin), source_element, cap_filter1, nvvidconv1, nvvidconv2, cap_filter, NULL);

      if (!gst_element_link(source_element, cap_filter1))
      {
        g_printerr("Elements source_element and cap_filter1 could not be linked. Exiting.\n");
        return NULL;
      }
      if(INPUTFORMAT_STR == "image/jpeg"){
        if (!gst_element_link(cap_filter1, jpgdec))
        {
          g_printerr("Elements cap_filter1 and jpgdec could not be linked. Exiting.\n");
          return NULL;
        }

        if (!gst_element_link(jpgdec, nvvidconv1))
        {
          g_printerr("Elements jpgdec and nvvidconv1 could not be linked. Exiting.\n");
          return NULL;
        }
      }
      else {
        if (!gst_element_link(cap_filter1, nvvidconv1))
        {
          g_printerr("Elements cap_filter1 and nvvidconv1 could not be linked. Exiting.\n");
          return NULL;
        }
      }

      if (!gst_element_link(nvvidconv1, nvvidconv2))
      {
        g_printerr("Elements nvvidconv1 and nvvidconv2 could not be linked. Exiting.\n");
        return NULL;
      }

      if (!gst_element_link(nvvidconv2, cap_filter))
      {
        g_printerr("Elements nvvidconv2 and cap_filter could not be linked. Exiting.\n");
        return NULL;
      }

      GstPad *gstpad = gst_element_get_static_pad(cap_filter, "src");

      gst_element_add_pad(bin, gst_ghost_pad_new("src", gstpad));
      gst_object_unref(gstpad);
    }
    else
    {
      source_element = gst_element_factory_make("uridecodebin", "uri-decode-bin");
      if (!source_element)
      {
        g_printerr("Failed to create uridecodebin.\n");
        gst_object_unref(bin); // Unref bin to avoid memory leak
        return NULL;
      }
      g_print("here\n");
      g_object_set(G_OBJECT(source_element), "uri", uri, NULL);
      g_signal_connect(G_OBJECT(source_element), "pad-added", G_CALLBACK(cb_newpad), bin);
      g_signal_connect(G_OBJECT(source_element), "child-added", G_CALLBACK(decodebin_child_added), bin);

      gst_bin_add(GST_BIN(bin), source_element);

      // Add a ghost pad for the bin
      GstPad *ghost_pad = gst_ghost_pad_new_no_target("src", GST_PAD_SRC);
      if (!ghost_pad)
      {
        g_printerr("Failed to create ghost pad.\n");
        gst_object_unref(bin); // Unref bin to avoid memory leak
        return NULL;
      }
      // g_print("Added ghost pad\n");
      gst_element_add_pad(bin, ghost_pad);
    }

    return bin;
  }

  //this is used to get the absolute path
  gchar *
  Odin::get_absolute_file_path(gchar *cfg_file_path, gchar *file_path)
  {
    // g_print("////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////\n");
    // g_print("cfg_file_path: %s\n", cfg_file_path);
    // g_print("file_path: %s\n", file_path);
    gchar abs_cfg_path[PATH_MAX + 1];
    gchar *abs_file_path;
    gchar *delim;

    if (file_path && file_path[0] == '/')
    {
      // g_print("abs_file_path: %s\n", file_path);
      return file_path;
    }

    if (!realpath(cfg_file_path, abs_cfg_path))
    {
      g_free(file_path);
      return NULL;
    }

    // Return absolute path of config file if file_path is NULL.
    if (!file_path)
    {
      abs_file_path = g_strdup(abs_cfg_path);
      // g_print("abs_file_path: %s\n", abs_file_path);
      return abs_file_path;
    }
    // g_print("abs_cfg_path: %s\n", abs_cfg_path);
    delim = g_strrstr(abs_cfg_path, "/");
    // g_print("delim: %s\n", delim);
    *(delim + 1) = '\0';
    // g_print("abs_cfg_path: %s\n", abs_cfg_path);
    abs_file_path = g_strconcat(abs_cfg_path, file_path, NULL);
    g_free(file_path);
    // g_print("abs_file_path: %s\n", abs_file_path);
    return abs_file_path;
  }

  //for setting up the tracker properties from the config file
  gboolean
  Odin::set_tracker_properties(GstElement *nvtracker)
  {
    gboolean ret = FALSE;
    GError *error = NULL;
    gchar **keys = NULL;
    gchar **key = NULL;
    GKeyFile *key_file = g_key_file_new();

    if (!g_key_file_load_from_file(key_file, TRACKER_CONFIG_FILE, G_KEY_FILE_NONE,
                                   &error))
    {
      g_printerr("Failed to load config file: %s\n", error->message);
      return FALSE;
    }

    keys = g_key_file_get_keys(key_file, CONFIG_GROUP_TRACKER, NULL, &error);
    CHECK_ERROR(error);

    for (key = keys; *key; key++)
    {
      if (!g_strcmp0(*key, CONFIG_GROUP_TRACKER_WIDTH))
      {
        gint width =
            g_key_file_get_integer(key_file, CONFIG_GROUP_TRACKER,
                                   CONFIG_GROUP_TRACKER_WIDTH, &error);
        CHECK_ERROR(error);
        g_object_set(G_OBJECT(nvtracker), "tracker-width", width, NULL);
      }
      else if (!g_strcmp0(*key, CONFIG_GROUP_TRACKER_HEIGHT))
      {
        gint height =
            g_key_file_get_integer(key_file, CONFIG_GROUP_TRACKER,
                                   CONFIG_GROUP_TRACKER_HEIGHT, &error);
        CHECK_ERROR(error);
        g_object_set(G_OBJECT(nvtracker), "tracker-height", height, NULL);
      }
      else if (!g_strcmp0(*key, CONFIG_GPU_ID))
      {
        guint gpu_id =
            g_key_file_get_integer(key_file, CONFIG_GROUP_TRACKER,
                                   CONFIG_GPU_ID, &error);
        CHECK_ERROR(error);
        g_object_set(G_OBJECT(nvtracker), "gpu_id", gpu_id, NULL);
      }
      else if (!g_strcmp0(*key, CONFIG_GROUP_TRACKER_LL_CONFIG_FILE))
      {
        char *ll_config_file = get_absolute_file_path(TRACKER_CONFIG_FILE,
                                                      g_key_file_get_string(key_file,
                                                                            CONFIG_GROUP_TRACKER,
                                                                            CONFIG_GROUP_TRACKER_LL_CONFIG_FILE, &error));
        CHECK_ERROR(error);
        g_object_set(G_OBJECT(nvtracker), "ll-config-file", ll_config_file, NULL);
      }
      else if (!g_strcmp0(*key, CONFIG_GROUP_TRACKER_LL_LIB_FILE))
      {
        char *ll_lib_file = get_absolute_file_path(TRACKER_CONFIG_FILE,
                                                   g_key_file_get_string(key_file,
                                                                         CONFIG_GROUP_TRACKER,
                                                                         CONFIG_GROUP_TRACKER_LL_LIB_FILE, &error));
        CHECK_ERROR(error);
        g_object_set(G_OBJECT(nvtracker), "ll-lib-file", ll_lib_file, NULL);
      }
      else if (!g_strcmp0(*key, CONFIG_GROUP_TRACKER_ENABLE_BATCH_PROCESS))
      {
        gboolean enable_batch_process =
            g_key_file_get_integer(key_file, CONFIG_GROUP_TRACKER,
                                   CONFIG_GROUP_TRACKER_ENABLE_BATCH_PROCESS, &error);
        CHECK_ERROR(error);
        // g_object_set(G_OBJECT(nvtracker), "enable_batch_process",
        //             enable_batch_process, NULL);
      }
      else
      {
        g_printerr("Unknown key '%s' for group [%s]", *key,
                   CONFIG_GROUP_TRACKER);
      }
    }

    ret = TRUE;

  done:
    if (error)
    {
      g_error_free(error);
    }
    if (keys)
    {
      g_strfreev(keys);
    }
    if (!ret)
    {
      g_printerr("%s failed", __func__);
    }
    return ret;
  }

  //to set up the properties of different plugins of the pipeline except the input sources
  int 
  Odin::configure_element_properties(int num_sources, GstElement *streammux, GstElement *pgie_yolo_detector,
                                         GstElement *nvtracker, GstElement *sink, GstElement *tiler)
  {

    guint tiler_rows, tiler_columns;

    g_object_set(G_OBJECT(streammux), "width", MUXER_OUTPUT_WIDTH,
                 "height", MUXER_OUTPUT_HEIGHT, "batch-size", num_sources,
                 "batched-push-timeout", MUXER_BATCH_TIMEOUT_USEC,
                 "live-source", TRUE, NULL);

    // Set all important properties of pgie_yolo_detector
    g_object_set(G_OBJECT(pgie_yolo_detector),
                 "config-file-path", PGIE_YOLO_DETECTOR_CONFIG_FILE_PATH, NULL);

    int batch_size = 0;

    if (num_sources <= 4)
    {
      batch_size = num_sources;
    }
    else
    {
      batch_size = 4;
    }

    // Override batch-size of pgie_yolo_detector
    g_object_set(G_OBJECT(pgie_yolo_detector), "batch-size", batch_size, NULL);

    // Check if Engine Exists
    if (boost::filesystem::exists(boost::filesystem::path(
            YOLOv8::Odin::PGIE_YOLO_ENGINE_PATH)))
    {

      g_object_set(G_OBJECT(pgie_yolo_detector),
                   "model-engine-file", YOLOv8::Odin::PGIE_YOLO_ENGINE_PATH.c_str(), NULL);
    }
    else
    {
      cout << str(boost::format("YOLO Engine for batch-size: %d and compute-mode: %s not found.") % batch_size % COMPUTE_MODE) << endl;
      return EXIT_FAILURE;
    }

    // Set necessary properties of the tracker element
    if (!YOLOv8::Odin::set_tracker_properties(nvtracker))
    {
      g_printerr("Failed to set tracker properties. Exiting.\n");
      return -1;
    }

    g_object_set(G_OBJECT(sink),
                 "sync", FALSE, NULL);

    tiler_rows = (guint)sqrt(num_sources);
    tiler_columns = (guint)ceil(1.0 * num_sources / tiler_rows);
    // Tiler Properties
    g_object_set(G_OBJECT(tiler), "rows", tiler_rows, "columns", tiler_columns,
                 "width", TILED_OUTPUT_WIDTH, "height", TILED_OUTPUT_HEIGHT, NULL);

    return EXIT_SUCCESS;
  }

  //to set paths for different plugins
  void Odin::setPaths(guint batch_size)
  {

    // Config Paths
    PGIE_YOLO_DETECTOR_CONFIG_FILE_PATH =
        strdup("models/YOLOv8/config_infer_primary_yolov8.txt");

    TRACKER_CONFIG_FILE =
        strdup("tracker/tracker_config.txt");

    // Engine Paths
    PGIE_YOLO_ENGINE_PATH =
        str(boost::format("models/YOLOv8/best.onnx_b%d_gpu0_%s.engine") % batch_size % COMPUTE_MODE);
  }
}

//main function call
int main(int argc, char *argv[])
{

  YOLOv8::Odin odin;
  GMainLoop *loop = NULL;
  GstElement *pipeline = NULL, *streammux = NULL, *sink = NULL,
             *pgie_yolo_detector = NULL, *nvtracker = NULL,
             *nvvidconv = NULL, *nvosd = NULL, *tiler;

#ifdef PLATFORM_TEGRA
  GstElement *transform = NULL;
#endif

  GstBus *bus = NULL;
  guint bus_watch_id = 0;
  GstPad *tiler_src_pad = NULL;
  guint i;

  // Standard GStreamer initialization
  gst_init(&argc, &argv);
  loop = g_main_loop_new(NULL, FALSE);

  GOptionContext *ctx = NULL;
  GOptionGroup *group = NULL;
  GError *error = NULL;

  ctx = g_option_context_new("Odin DeepStream App");
  group = g_option_group_new("Odin", NULL, NULL, NULL, NULL);
  g_option_group_add_entries(group, odin.entries);

  g_option_context_set_main_group(ctx, group);
  g_option_context_add_group(ctx, gst_init_get_option_group());

  if (!g_option_context_parse(ctx, &argc, &argv, &error))
  {
    g_option_context_free(ctx);
    g_printerr("%s", error->message);
    return -1;
  }
  g_option_context_free(ctx);

  /* Create gstreamer elements */
  // Create Pipeline element to connect all elements
  pipeline = gst_pipeline_new("dsirisretail-pipeline");

  // Stream Multiplexer for input
  streammux = gst_element_factory_make("nvstreammux", "stream-muxer");

  if (!pipeline || !streammux)
  {
    g_printerr("One element could not be created. Exiting.\n");
    return -1;
  }
  gst_bin_add(GST_BIN(pipeline), streammux);
  gint sources = odin.create_input_sources(pipeline, streammux, num_sources);
  if (sources == -1)
  {
    return -1;
  }
  else
  {
    num_sources = sources;
  }

  int batch_size = 0;
  if (num_sources <= 4)
  {
    batch_size = num_sources;
  }
  else
  {
    batch_size = 4;
  }
  odin.setPaths(batch_size);

  // Primary GPU Inference Engine
  pgie_yolo_detector = gst_element_factory_make("nvinfer", "primary-yolo-nvinference-engine");

  if (!pgie_yolo_detector)
  {
    g_printerr("PGIE YOLO Detector could not be created.\n");
    return -1;
  }

  // Initialize Tracker
  nvtracker = gst_element_factory_make("nvtracker", "tracker");

  if (!nvtracker)
  {
    g_printerr("NVTRACKER could not be created.\n");
    return -1;
  }

  // Compose all the sources into one 2D tiled window
  tiler = gst_element_factory_make("nvmultistreamtiler", "nvtiler");

  if (!tiler)
  {
    g_printerr("SINK could not be created.\n");
    return -1;
  }

  // Use convertor to convert from NV12 to RGBA as required by nvosd
  nvvidconv = gst_element_factory_make("nvvideoconvert", "nvvideo-converter");

  if (!nvvidconv)
  {
    g_printerr("NVVIDCONV could not be created.\n");
    return -1;
  }

  // Create OSD to draw on the converted RGBA buffer
  nvosd = gst_element_factory_make("nvdsosd", "nv-onscreendisplay");

  if (!nvosd)
  {
    g_printerr("NVOSD could not be created.\n");
    return -1;
  }
/* Redner OSD Output */
#ifdef PLATFORM_TEGRA
  transform = gst_element_factory_make("nvegltransform", "nvegl-transform");
#endif

  if (odin.display_off)
  {
    sink = gst_element_factory_make("fakesink", "nvvideo-renderer");
  }
  else
  {
    sink = gst_element_factory_make("nveglglessink", "nvvideo-renderer");
  }

  if (!sink)
  {
    g_printerr("SINK could not be created.\n");
    return -1;
  }

#ifdef PLATFORM_TEGRA
  if (!transform)
  {
    g_printerr("Tegra element TRANSFORM could not be created. Exiting.\n");
    return -1;
  }
#endif

  int fail_safe = odin.configure_element_properties(num_sources, streammux, pgie_yolo_detector,
                                                    nvtracker, sink, tiler);

  if (fail_safe == -1)
  {
    return -1;
  }
  // Message Handler
  bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  bus_watch_id = gst_bus_add_watch(bus, odin.bus_call, loop);
  gst_object_unref(bus);

/* Set up the pipeline */
#ifdef PLATFORM_TEGRA
  if (!odin.display_off)
  {
    gst_bin_add_many(GST_BIN(pipeline),
                     pgie_yolo_detector, nvtracker, tiler, nvvidconv, nvosd, transform, sink, NULL);

    if (!gst_element_link_many(streammux, pgie_yolo_detector, nvtracker,
                               tiler, nvvidconv, nvosd, transform, sink, NULL))
    {
      g_printerr("Elements could not be linked. Exiting.\n");
      return -1;
    }
  }
  else
  {
    gst_bin_add_many(GST_BIN(pipeline),
                     pgie_yolo_detector, nvtracker, tiler, nvvidconv, nvosd, sink, NULL);

    if (!gst_element_link_many(streammux, pgie_yolo_detector, nvtracker,
                               tiler, nvvidconv, nvosd, sink, NULL))
    {
      g_printerr("Elements could not be linked. Exiting.\n");
      return -1;
    }
  }

#else
  gst_bin_add_many(GST_BIN(pipeline),
                   pgie_yolo_detector, nvtracker, tiler, nvvidconv, nvosd, sink, NULL);

  if (!gst_element_link_many(streammux, pgie_yolo_detector, nvtracker,
                             tiler, nvvidconv, nvosd, sink, NULL))
  {
    g_printerr("Elements could not be linked. Exiting.\n");
    return -1;
  }
#endif

  /* Lets add probe to get informed of the meta data generated, we add probe to
   * the sink pad of the osd element, since by that time, the buffer would have
   * had got all the metadata. */
  tiler_src_pad = gst_element_get_static_pad(tiler, "sink");
  if (!tiler_src_pad)
  {
    g_print("Unable to get sink pad\n");
  }
  else
  {
    gst_pad_add_probe(tiler_src_pad, GST_PAD_PROBE_TYPE_BUFFER,
                      odin.tiler_src_pad_buffer_probe, NULL, NULL);
  }
  /* Set the pipeline to "playing" state */
  cout << "Now playing:" << endl;
  std::ifstream infile(SOURCE_PATH);
  std::string source;
  if (infile.is_open())
  {
    while (getline(infile, source))
    {
      if (source.empty() || source[0] == '#')
      {
        g_print("Skipping line: %s\n", source.c_str());
        continue;
      }
      cout << source << endl;
    }
  }
  infile.close();

  gst_element_set_state(pipeline, GST_STATE_PLAYING);

  /* Wait till pipeline encounters an error or EOS */
  g_print("Running...\n");
  g_main_loop_run(loop);
  /* Out of the main loop, clean up nicely */
  g_print("Returned, stopping playback\n");
  gst_element_set_state(pipeline, GST_STATE_NULL);
  g_print("Deleting pipeline\n");
  gst_object_unref(GST_OBJECT(pipeline));
  g_source_remove(bus_watch_id);
  g_main_loop_unref(loop);
  return 0;
}