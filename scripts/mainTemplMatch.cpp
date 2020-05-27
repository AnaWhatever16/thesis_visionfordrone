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

#include <aerox_suite/hal/RealSense/RealSenseCamera.h>
#include <aerox_suite/state_estimation/algorithms/TemplateMatching.h>

int main(int _argc, char **_argv){

    RealSenseCamera realSenseCamera(0);
    TemplateMatching templateMatching(_argv[1]);

    if(templateMatching.templ.rows == 0 ) { // Check for invalid template
        std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    while(true){
        realSenseCamera.setImage();
        templateMatching.matchingMethod(realSenseCamera.img);
        cv::waitKey(3);
    }
    cv::destroyAllWindows();
    return 0;
}