#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <image_consumer.hpp>


void ImageConsumer::ImagePipeline(int width, int height, int fps) {

    rs2::colorizer color_map;
    
    // Declare RealSense pipeline, encapsulating the actual device and sensors 
    rs2::pipeline p;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, fps);

    // Start streaming with default recommended configuration

    p.start(cfg);

    for (int i = 0; i < 30; i++)
    {
        p.wait_for_frames();
    } 

    std::cout << "Recording..." << std::endl;

    for (auto a = 1; a < 300; a++)
    {
        rs2::frameset data = p.wait_for_frames();
        rs2::frame color = data.get_color_frame();

        auto stream = color.get_profile().stream_type();

        // Write image to dgitisk
        std::stringstream png_file;
        png_file << "data/img_" << std::setfill('0') << std::setw(3) << a << ".png";
        stbi_write_png(png_file.str().c_str(), color.as<rs2::video_frame>().get_width(), color.as<rs2::video_frame>().get_height(), 
                        color.as<rs2::video_frame>().get_bytes_per_pixel(), (void*)color.get_data(), color.as<rs2::video_frame>().get_stride_in_bytes());
        // std::cout << "Saved " << png_file.str() << std::endl;

        // Record per-frame metadata for UVC streams
        // std::stringstream csv_file;
        // csv_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << "-metadata.csv";
        // metadata_to_csv(vf, csv_file.str());        
    }

}

void ImageConsumer::ImageWindow(rs2::frame& frame, std::string& window_name)
{
    
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
