#include <aerox_vision/algorithms/KCFTracker.h>
#include <opencv2/core.hpp>
#include <iostream>

int main(int _argc, char **_argv){

    cv::VideoCapture camera(0);
    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    cv::Mat frame;
    camera >> frame;

    aerox::KCFTracker tracker(frame, 0, "/home/ana/programming/Aerox_Suite/samples/algorithm_tests/templ1.jpg"); // change path to image

    while((char)27!=cv::waitKey(1)){
        camera >> frame;
        if(!tracker.update(frame)){
            std::cout << "The target has been lost..."<< std::endl;
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}