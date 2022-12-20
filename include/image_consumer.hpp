#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
#include <iostream>

void metadata_to_csv(const rs2::frame& frm, const std::string& filename);

class ImageConsumer{
    public:
        ImageConsumer()
        {
            
        }
        void ImagePipeline(int width = 1280, int height = 720, int fps = 30);
        void ImageProcesser(float sensor[6]);
        
};