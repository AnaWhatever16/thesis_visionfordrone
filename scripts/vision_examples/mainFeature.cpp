#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <realsense/RealSenseCamera.h>
#include <vision/FeatureMatching.h>

int main(int _argc, char **_argv){

    cv::VideoCapture camera(0);
    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    cv::Mat frame;
    camera >> frame;
    FeatureMatching featureDetection(frame);

    //RealSenseCamera realSenseCamera(RealSenseCamera::RS_TYPE::D435);
    //FeatureMatching featureDetection(_argv[1]);
    //cv::Mat frame;

    while((char)27!=cv::waitKey(1)){
        camera>>frame;
        //realSenseCamera>>frame;
        featureDetection.method(frame);
    }
    cv::destroyAllWindows();
    return 0;
}