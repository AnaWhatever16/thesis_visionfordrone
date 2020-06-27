#include <realsense/RealSenseCamera.h>
#include <opencv2/core.hpp>

int main(int _argc, char **_argv){
    RealSenseCamera realSense(RealSenseCamera::RS_TYPE::D435);
    cv::Mat frame;
    while((char)27!=cv::waitKey(1)){
        realSense>>frame;
        imshow("Camera Example", frame);
    }

    cv::imwrite("/home/ana/Desktop/targetVision.jpg", frame);    
    return 0;

}