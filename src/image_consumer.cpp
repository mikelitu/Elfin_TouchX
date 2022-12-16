#include "image_consumer.hpp"

//Window config
#define VIDEO_WIDTH  640
#define VIDEO_HEIGHT 480
#define WINDOW_NAME "Param Adjust"

//Image buffer size
#define BUFFER_SIZE 1

volatile unsigned int prdIdx = 0; //image reading index
volatile unsigned int csmIdx = 0; //image processing index

//image processing speed output
double time_process;
char process_time[30];
char timenow[20];

struct ImageData {
	cv::Mat img;             //camare data
	unsigned int frame;  //frame count
};

ImageData capturedata[BUFFER_SIZE];   //buffer of capture


void ImageConsumer::ImageReader()
{
	//open camare
	cv::VideoCapture cap(2);
	cap.set(cv::CAP_PROP_FRAME_WIDTH,1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,1080);
	if (!cap.isOpened())
    {
            std::cout<<"can't open video or cam"<<std::endl;
            return;
	}
		
	while(true)
    {
        //wait for next image
       	while(prdIdx - csmIdx >= BUFFER_SIZE);
        cap >> capturedata[prdIdx % BUFFER_SIZE].img;
        capturedata[prdIdx % BUFFER_SIZE].frame++; 	//frame is the index of picture
        ++prdIdx;
    }
}

void ImageConsumer::ImageProcesser(float sensor[6]) 
{
    cv::Mat frame;

    while(true)
    {
        for (int i = 0; i < 6; i++)
        {
            std::cout << sensor[i] << ",";
        }
        std::cout << std::endl;
        while (prdIdx - csmIdx == 0);
		capturedata[csmIdx % BUFFER_SIZE].img.copyTo(frame);    // get images from buffer
		++csmIdx;
        frame = frame(cv::Range(200, 800), cv::Range(600, 1000));
        imshow("frame", frame); 

        cv::Mat rgb;
        cv::cvtColor(frame,rgb,cv::COLOR_BGR2RGB);


        char key = cv::waitKey(10);
		if(key == 27)
			exit(0);
    }
}
