#pragma once
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unistd.h>


class ImageConsumer{
    public:
        ImageConsumer()
        {
            
        }
        void ImagePipeline(int width, int height, int fps, bool& init_exp, std::string& save_dir);
        void ImageWindow();
        
};