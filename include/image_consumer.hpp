#pragma once
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <Eigen/Dense>
#include <robot_control.hpp>


class ImageConsumer{
    public:
        ImageConsumer()
        {
            
        }
        void ImagePipeline(int width, int height, int fps);
        void ImageWindow();
        
};