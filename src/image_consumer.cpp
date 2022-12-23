#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <image_consumer.hpp>

#define BUFFER_SIZE 1

volatile unsigned int prdIdx = 0;
volatile unsigned int csmIdx = 0;
// Saving image tracker
volatile unsigned int i = 0;


// Image buffer to keep track of images and update the view
struct ImageData {
    cv::Mat img;
    unsigned int frame; //frame counter
};

ImageData capturedata[BUFFER_SIZE];

/**
 * @brief Realsense camera image pipeline and disk-saving function, based on the librealsense tutorial
 * (https://github.com/IntelRealSense/librealsense/tree/master/examples/save-to-disk)
 * 
 * @param width Image width to initialize the Realsense pipeline (max. 1280)
 * @param height Image height to initialize the Realsense pipeline (max. 720)
 * @param fps Video fps to initialize the Realsense pipeline (max 90)
 */

void ImageConsumer::ImagePipeline(int width, int height, int fps, bool& init_exp) {
    
    // Declare RealSense pipeline, encapsulating the actual device and sensors 
    rs2::pipeline p;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, fps);


    // Start streaming with the defined parameters
    p.start(cfg);

    while(true)
    {
        rs2::frameset data = p.wait_for_frames();
        rs2::frame color = data.get_color_frame();

        while(prdIdx - csmIdx >= BUFFER_SIZE);
        cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        capturedata[prdIdx % BUFFER_SIZE].img = image;
        capturedata[prdIdx % BUFFER_SIZE].frame++;
        if (init_exp)
        {
            std::stringstream png_file;
            png_file << "data/img_" << std::setfill('0') << std::setw(4) << i << ".png";
            stbi_write_png(png_file.str().c_str(), color.as<rs2::video_frame>().get_width(), color.as<rs2::video_frame>().get_height(), 
                            color.as<rs2::video_frame>().get_bytes_per_pixel(), (void*)color.get_data(), color.as<rs2::video_frame>().get_stride_in_bytes());
            i++;
        }
        ++prdIdx;
    }

}

/**
 * @brief Function to display Realsense camera using OpenCV, based on the librealsense tutorial 
 * (https://github.com/IntelRealSense/librealsense/blob/master/wrappers/opencv/imshow/rs-imshow.cpp)
 * 
 */
void ImageConsumer::ImageWindow()
{
    cv::Mat frame;
    const std::string window_name = "Display Image";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    while (true)
    {
        while (prdIdx - csmIdx == 0);
        capturedata[csmIdx % BUFFER_SIZE].img.copyTo(frame);
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
        ++csmIdx;
        cv::imshow(window_name, frame);
        char key = cv::waitKey(10);
        if (key == 27) exit(0);
    }
}

