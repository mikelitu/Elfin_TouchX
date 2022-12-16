#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

class ImageConsumer{
    public:
        ImageConsumer()
        {
            
        }
        void ImageReader();
        void ImageProcesser(float sensor[6]);
        
};