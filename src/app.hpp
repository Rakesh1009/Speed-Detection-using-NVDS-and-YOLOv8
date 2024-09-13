#pragma once
#include "libs.hpp"

namespace YOLOv8
{

#include <vector>
#include <stdexcept>
#include <iostream>

    template <typename T>
    class CircularBuffer
    {
    public:
        CircularBuffer(size_t size)
            : size(size), buffer(size), sum(0), count(0), head(0)
        {
            if (size == 0)
            {
                throw std::invalid_argument("Buffer size must be greater than 0");
            }
            initialize();
        }

        void addValue(T value)
        {
            if (count < size)
            {
                count++;
            }
            else
            {
                sum -= buffer[head];
            }
            sum += value;
            buffer[head] = value;
            head = (head + 1) % size;
        }

        double getAverage() const
        {   
                    if (count == 0)
            {
                return 0; // Avoid division by zero
            }

            double weighted_sum = 0;
            double weight_total = 0;

            for (size_t i = 0; i < count; ++i)
            {
                // Calculate position in the buffer for each value
                size_t pos = (head + size - count + i) % size;
                
                // Linear weighting: More recent values have higher weight
                double weight = static_cast<double>(i + 1);
                // weight = weight*weight;  // Weight increases with `i + 1`
                weighted_sum += buffer[pos] * weight;
                weight_total += weight;
            }

            // Return the weighted average
            return weighted_sum / weight_total;
        }

        vector<T> getValues()
        {
            vector<T> values;
            for (size_t i = 0; i < count; ++i)
            {
                size_t pos = (head + size - count + i) % size;
                values.push_back(buffer[pos]);
            }
            return values;
        }

        T getMostRecentValue() const
        {
            // `head` points to the next position to be filled, so the most recent value
            // is at the previous position: (head + size - 1) % size
            size_t most_recent_index = (head + size - 1 + size) % size;
            return buffer[most_recent_index];
        }

        T &operator[](size_t index)
        {
            // Calculate the position in the buffer
            size_t pos = (head + size - count + index) % size;
            return buffer[pos];
        }

        const T &operator[](size_t index) const
        {
            // Calculate the position in the buffer
            size_t pos = (head + size - count + index) % size;
            return buffer[pos];
        }

        void printQueue() const
        {
            for (size_t i = 0; i < count; ++i)
            {
                size_t pos = (head + size - count + i) % size;
                std::cout << buffer[pos] << " "; // Adjust format specifier for your type
            }
            std::cout << "\n";
        }

    private:
        size_t size;
        std::vector<T> buffer;
        double sum;
        size_t count;
        size_t head;

        void initialize()
        {
            // Default behavior for types other than float, int, or double
            // No additional initialization needed for other types
            std::fill(buffer.begin(), buffer.end(), T{});
        }
    };

    // Specialization for floating-point types
    template <>
    void CircularBuffer<float>::initialize()
    {
        std::fill(buffer.begin(), buffer.end(), 0.0f);
    }

    template <>
    void CircularBuffer<double>::initialize()
    {
        std::fill(buffer.begin(), buffer.end(), 0.0);
    }

    // Specialization for int type
    template <>
    void CircularBuffer<int>::initialize()
    {
        std::fill(buffer.begin(), buffer.end(), 0);
    }
    struct ObjectData
    {
        CircularBuffer<float> prev_x;
        CircularBuffer<float> prev_y;
        bool first_frame;
        CircularBuffer<float> speeds;
        float last_x;
        float last_y;
        float last_d;
        std::chrono::system_clock::time_point last_t;

        ObjectData()
            : prev_x(TRAIL_LEN),
              prev_y(TRAIL_LEN),
              first_frame(true),
              speeds(NUM_SPEED),
              last_x(0.0f),
              last_y(0.0f),
              last_d(0.0f),
              last_t(std::chrono::system_clock::now())
        {
        }
    };

    class Odin
    {
    private:
        struct fps_calculator
        {
            system_clock::time_point fps_timer;
            system_clock::time_point display_timer;
            gint rolling_fps;
            gint display_fps;
        };

        inline static fps_calculator fps[16];

        inline static char *PGIE_YOLO_DETECTOR_CONFIG_FILE_PATH;

        inline static char *TRACKER_CONFIG_FILE;

        static float distance;

    public:
        // To save the frames
        gint frame_number;

        static gboolean display_off;

        static CircularBuffer<ObjectData> objectdatamap;

        GOptionEntry entries[2] = {
            {"no-display", 0, 0, G_OPTION_ARG_NONE, &display_off, "Disable display", NULL},
            {NULL}};

        std::string PGIE_YOLO_ENGINE_PATH;

        static void
        update_fps(gint id);

        static int
        create_input_sources(gpointer pipe, gpointer mux, guint num_sources);

        static void
        changeBBoxColor(gpointer obj_meta_data, int has_bg_color, float red, float green,
                        float blue, float alpha);

        static void
        addDisplayMeta(gpointer batch_meta_data, gpointer frame_meta_data);

        static GstPadProbeReturn
        tiler_src_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info,
                                   gpointer u_data);

        static gboolean
        bus_call(GstBus *bus, GstMessage *msg, gpointer data);

        static void
        cb_newpad(GstElement *decodebin, GstPad *decoder_src_pad, gpointer data);

        static void
        decodebin_child_added(GstChildProxy *child_proxy, GObject *object,
                              gchar *name, gpointer user_data);

        static GstElement *
        create_source_bin(guint index, gchar *uri);

        static gchar *
        get_absolute_file_path(gchar *cfg_file_path, gchar *file_path);

        static gboolean
        set_tracker_properties(GstElement *nvtracker);

        int
        configure_element_properties(int num_sources, GstElement *streammux, GstElement *pgie_yolo_detector,
                                     GstElement *nvtracker, GstElement *sink, GstElement *tiler);

        void
        setPaths(guint num_sources);

        static double
        calculate_object_speed(NvDsObjectMeta *obj_meta);


        static pair<float, float>
        convert_coordinates(float u, float v);

        Odin()
        {

            int counter;
            for (counter = 0; counter < 16; counter++)
            {
                fps[counter].fps_timer = system_clock::now();
                fps[counter].display_timer = system_clock::now();
                fps[counter].rolling_fps = 0;
                fps[counter].display_fps = 0;
            }
            frame_number = 0;
        }
    };
}