#include <opencv2/highgui.hpp>
#include <iostream>

#include <realsense/RealSenseCamera.h>
#include <vision/TemplateMatchThread.h>

int main(int _argc, char **_argv){

    RealSenseCamera realSenseCamera(RealSenseCamera::RS_TYPE::D435);
    TemplateMatchThread templateMatchThread(_argv[1]);

    //int match_method=cv::TM_SQDIFF
    //int match_method=cv::TM_SQDIFF_NORMED;
    //int match_method=cv::TM_CCORR;
    //int match_method=cv::TM_CCORR_NORMED
    //int match_method=cv::TM_CCOEFF;
    int match_method=cv::TM_CCOEFF_NORMED;

    cv::Mat frame;

    while((char)27!=cv::waitKey(1)){
        realSenseCamera>>frame;
        templateMatchThread.matchThread(frame, match_method);
    }

    cv::destroyAllWindows();    
    return 0;
}