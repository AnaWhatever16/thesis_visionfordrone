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

#include <aerox_vision/algorithms/LucasMethod.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>

namespace aerox{
    LucasMethod::LucasMethod(cv::Mat &_input){
        //Create some random colors
        cv::RNG rng;
        for(int i = 0; i < 100; i++){
            int r = rng.uniform(0, 256);
            int g = rng.uniform(0, 256);
            int b = rng.uniform(0, 256);
            colors.push_back(cv::Scalar(r,g,b));
        }
        // Take first frame and find corners in it
        old_frame=_input;
        cv::cvtColor(old_frame, old_gray, CV_BGR2GRAY);
        cv::goodFeaturesToTrack(old_gray, p0, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);
        // Create a mask image for drawing purposes
        mask = cv::Mat::zeros(old_frame.size(), old_frame.type());
        criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 50, 0.03); //originally 10, 0.03
    }

    void LucasMethod::method(cv::Mat &_input){
        cv::Mat frame, frame_gray;
        frame=_input;
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
        // calculate optical flow
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, cv::Size(15,15), 2, criteria);
        std::vector<cv::Point2f> good_new;
        for(uint i = 0; i < p0.size(); i++)
        {
            // Select good points
            if(status[i] == 1) {
                good_new.push_back(p1[i]);
                // draw the tracks
                cv::line(mask,p1[i], p0[i], colors[i], 2);
                cv::circle(frame, p1[i], 5, colors[i], -1);
            }
        }
        cv::Mat img;
        cv::add(frame, mask, img);
        cv::imshow("Frame", img);
        // Now update the previous frame and previous points
        old_gray = frame_gray.clone();
        p0 = good_new;
    }
}