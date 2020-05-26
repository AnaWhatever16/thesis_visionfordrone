//---------------------------------------------------------------------------------------------------------------------
//  Vertical Engineering Solutions
//---------------------------------------------------------------------------------------------------------------------
// 
//  Copyright 2020 Vertical Engineering Solutions  - All Rights Reserved
// 
//  Unauthorized copying of this file, via any medium is strictly prohibited Proprietary and confidential.
// 
//  All information contained herein is, and remains the property of Vertical Engineering Solutions.  The 
//  intellectual and technical concepts contained herein are proprietary to Vertical Engineering Solutions 
//  and its suppliers and may be covered by UE and Foreign Patents, patents in process, and are protected 
//  by trade secret or copyright law. Dissemination of this information or reproduction of this material is 
//  strictly forbidden unless prior written permission is obtained from Adobe Systems Incorporated.
//
//---------------------------------------------------------------------------------------------------------------------
//
//  Maintainer: acasado@vengineerings.com
//
//---------------------------------------------------------------------------------------------------------------------

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <aerox_suite/hal/RealSense/RealSenseCamera.h>
#include <aerox_suite/state_estimation/algorithms/FeatureMatching.h>

int main(int _argc, char **_argv){

    RealSenseCamera realSenseCamera(0);
    FeatureMatching featureDetection(_argv[1]);

   if (featureDetection.templ_.empty() )
    {
        std::cout << "Could not open or find the image!\n" << std::endl;
        std::cout << "Usage: " << _argv[0] << " <Input image>" << std::endl;
        return -1;
    }
    // std::vector<cv::KeyPoint> keypoints_img;
    // cv::Mat descriptors_img;

    while(true){
        realSenseCamera.setImage();
        featureDetection.method(realSenseCamera.img);
        //featureDetection.detection(realSenseCamera.img, keypoints_img, descriptors_img);
        // //-- Draw keypoints
        // cv::Mat img_keypoints;
        // drawKeypoints(realSenseCamera.img, keypoints_img, img_keypoints);
        // //-- Show detected (drawn) keypoints
        // imshow("ORB Keypoints", img_keypoints );
        //featureDetection.matching(descriptors_img, keypoints_img, realSenseCamera.img);
        cv::waitKey(3);
    }
    cv::destroyAllWindows();
    return 0;
}