#include <iostream>
#include <unistd.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <realsense/RealSenseCamera.h>
#include <vision/KCFTracker.h>

int main(int _argc, char **_argv){

    RealSenseCamera realSenseCamera(RealSenseCamera::RS_TYPE::D435);
    cv::Mat frame;

    std::cout << "Press ESC when image is ready for detection" << std::endl;

    while((char)27!=cv::waitKey(1)){
        realSenseCamera>>frame;
        cv::imshow("RealSense Image", frame); 
    }
    cv::destroyWindow("RealSense Image");

    KCFTracker tracker(frame);

    while((char)27!=cv::waitKey(1)){
        realSenseCamera>>frame;

        if(!tracker.update(frame)){
            std::cout << "The target has been lost..."<< std::endl;
            break;
        }

    }

    cv::destroyAllWindows();
    return 0;
}