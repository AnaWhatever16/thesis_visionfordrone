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

#include <opencv2/highgui.hpp>
#include <iostream>

#include <realsense/RealSenseCamera.h>
#include <vision/LucasMethod.h>

int main(int _argc, char **_argv){
    cv::Mat frame;
    RealSenseCamera realSenseCamera(RealSenseCamera::RS_TYPE::D435);
    
    std::cout << "Press ESC when image is ready for detection" << std::endl;

    while((char)27!=cv::waitKey(1)){
        realSenseCamera>>frame;
        cv::imshow("RealSense Image", frame); 
    }
    cv::destroyWindow("RealSense Image");

    LucasMethod lucas(frame);

    while((char)27!=cv::waitKey(1)){
        realSenseCamera>>frame;
        lucas.method(frame);
    }
    cv::destroyAllWindows();
    return 0;
}