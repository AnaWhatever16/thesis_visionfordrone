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

#include <realsense/RealSenseCamera.h>
#include <vision/FeatureMatching.h>

int main(int _argc, char **_argv){

    RealSenseCamera realSenseCamera(RealSenseCamera::RS_TYPE::D435);
    FeatureMatching featureDetection(_argv[1]);
    cv::Mat frame;

    while((char)27!=cv::waitKey(1)){
        realSenseCamera>>frame;
        featureDetection.method(frame);
    }
    cv::destroyAllWindows();
    return 0;
}