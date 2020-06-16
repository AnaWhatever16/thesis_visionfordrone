#include <iostream>
#include "opencv2/core.hpp"
#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <realsense/RealSenseCamera.h>
using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;
int main( int argc, char* argv[] )
{
    RealSenseCamera realSenseCamera(RealSenseCamera::RS_TYPE::D435);
    cv::Mat frame;
    while((char)27!=cv::waitKey(1)){
        realSenseCamera>>frame;
        //-- Step 1: Detect the keypoints using SURF Detector
        int minHessian = 400;
        Ptr<SURF> detector = SURF::create( minHessian );
        std::vector<KeyPoint> keypoints;
        detector->detect( frame, keypoints );
        //-- Draw keypoints
        Mat img_keypoints;
        drawKeypoints( frame, keypoints, img_keypoints );
        //-- Show detected (drawn) keypoints
        imshow("SURF Keypoints", img_keypoints );
        waitKey(3);
    }
    return 0;
}
#else
int main()
{
    std::cout << "This tutorial code needs the xfeatures2d contrib module to be run." << std::endl;
    return 0;
}
#endif