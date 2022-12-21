#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <image_consumer.hpp>

#define BUFFER_SIZE 1

volatile unsigned int prdIdx = 0;
volatile unsigned int csmIdx = 0;
volatile unsigned int i = 0;

// image processing spped output
double time_process;
char process_time[30];
char timenow[20];

struct ImageData {
    cv::Mat img;
    unsigned int frame; //frame counter
};

ImageData capturedata[BUFFER_SIZE];



void ImageConsumer::ImagePipeline(int width, int height, int fps) {
    
    // Declare RealSense pipeline, encapsulating the actual device and sensors 
    rs2::pipeline p;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, fps);


    // Start streaming with default recommended configuration

    p.start(cfg);

    while(true)
    {
        rs2::frameset data = p.wait_for_frames();
        rs2::frame color = data.get_color_frame();

        while(prdIdx - csmIdx >= BUFFER_SIZE);
        cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        capturedata[prdIdx % BUFFER_SIZE].img = image;
        capturedata[prdIdx % BUFFER_SIZE].frame++;
        if (prdIdx >= 30)
        {
            std::stringstream png_file;
            png_file << "data/img_" << std::setfill('0') << std::setw(3) << i << ".png";
            stbi_write_png(png_file.str().c_str(), color.as<rs2::video_frame>().get_width(), color.as<rs2::video_frame>().get_height(), 
                            color.as<rs2::video_frame>().get_bytes_per_pixel(), (void*)color.get_data(), color.as<rs2::video_frame>().get_stride_in_bytes());
            i++;
        }
        ++prdIdx;
    }

}

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

void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
{
    std::ofstream csv;

    csv.open(filename);

    csv << " Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Atribute,Value\n";

    // Record all the available metadata values
    for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
    {
        if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
        {
            csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
                << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
        }
    }

    csv.close();
}
