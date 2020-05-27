#include <opencv2/highgui.hpp>
#include <iostream>

#include <realsense/RealSenseCamera.h>
#include <vision/TemplateMatching.h>
#include <Eigen/Eigen> 

int main(int _argc, char **_argv){

    RealSenseCamera realSenseCamera(RealSenseCamera::RS_TYPE::D435);
    TemplateMatching templateMatching(_argv[1]);
    cv::Mat frame;

    //int match_method=cv::TM_SQDIFF
    //int match_method=cv::TM_SQDIFF_NORMED;
    //int match_method=cv::TM_CCORR;
    //int match_method=cv::TM_CCORR_NORMED
    //int match_method=cv::TM_CCOEFF;
    int match_method=cv::TM_CCOEFF_NORMED;

    while((char)27!=cv::waitKey(1)){
        realSenseCamera>>frame;
        templateMatching.matchingMethod(frame, match_method);
    }

    cv::destroyAllWindows();
    return 0;
}