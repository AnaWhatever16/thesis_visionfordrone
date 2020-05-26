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

#include <aerox_vision/algorithms/FarnebackMethod.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

namespace aerox{
    FarnebackMethod::FarnebackMethod(cv::Mat &_input){
        cv::Mat frame1=_input;
        cv::cvtColor(frame1, prvs, CV_BGR2GRAY);
    }

    void FarnebackMethod::method(cv::Mat &_input)
    {
        cv::Mat frame2, next;
        frame2=_input;
        cv::cvtColor(frame2, next, CV_BGR2GRAY);
        cv::Mat flow(prvs.size(), CV_32FC2);
        cv::calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
        // visualization
        cv::Mat flow_parts[2];
        cv::split(flow, flow_parts);
        cv::Mat magnitude, angle, magn_norm;
        cv::cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
        cv::normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
        angle *= ((1.f / 360.f) * (180.f / 255.f));
        //build hsv image
        cv::Mat _hsv[3], hsv, hsv8, bgr;
        _hsv[0] = angle;
        _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
        _hsv[2] = magn_norm;
        cv::merge(_hsv, 3, hsv);
        hsv.convertTo(hsv8, CV_8U, 255.0);
        cv::cvtColor(hsv8, bgr, CV_HSV2BGR);
        cv::imshow("frame2", bgr);
        cv::waitKey(30);
        prvs = next;
    }
}