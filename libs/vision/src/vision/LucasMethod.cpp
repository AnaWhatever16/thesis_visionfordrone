#include <vision/LucasMethod.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/tracking.hpp>

LucasMethod::LucasMethod(cv::Mat &_input){
    //Create some random colors
    cv::RNG rng;
    for(int i = 0; i < 100; i++){
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors_.push_back(cv::Scalar(r,g,b));
    }
    // Take first frame and find corners in it
    oldFrame_=_input;
    std::vector<cv::Point2f> p0Aux;
    cv::cvtColor(oldFrame_, oldGray_, cv::COLOR_BGR2GRAY);
    cv::goodFeaturesToTrack(oldGray_, p0Aux, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);
    p0_ = goodTrackingFeatures(p0Aux);
    // Create a mask image for drawing purposes
    mask_ = cv::Mat::zeros(oldFrame_.size(), oldFrame_.type());
    criteria_ = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 50, 0.03); //originally 10, 0.03
}

void LucasMethod::method(cv::Mat &_input){
    cv::Mat frame, frame_gray;
    std::vector<cv::Point2f> p1;
    frame=_input;
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    // calculate optical flow
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(oldGray_, frame_gray, p0_, p1, status, err, cv::Size(15,15), 2, criteria_);
    std::vector<cv::Point2f> good_new;
    for(uint i = 0; i < p0_.size(); i++)
    {
        // Select good points
        if(status[i] == 1) {
            good_new.push_back(p1[i]);
            // draw the tracks
            cv::line(mask_,p1[i], p0_[i], colors_[i], 2);
            cv::circle(frame, p1[i], 5, colors_[i], -1);
        }
    }

    cv::Mat img;
    cv::add(frame, mask_, img);
    drawBoundBox(good_new, img);
    cv::imshow("Frame", img);

    // Now update the previous frame and previous points
    oldGray_ = frame_gray.clone();
    p0_ = good_new;
}

std::vector<cv::Point2f> LucasMethod::goodTrackingFeatures(std::vector<cv::Point2f> &_p0){
    std::vector<cv::Point2f> pROI;
    cv::Rect2d roi = cv::selectROI("ROI", oldFrame_, true, false);

    while(roi.width==0 || roi.height==0){
        std::cout << "Try again" << std::endl;
        roi = selectROI("ROI", oldFrame_, false);
    }

    cv::destroyWindow("ROI");

    cv::Point2f tl = roi.tl();
    cv::Point2f br = roi.br(); 
    for (size_t i =0; i<_p0.size();i++){
        if (_p0[i].x >= tl.x && _p0[i].x <= br.x && _p0[i].y >= tl.y && _p0[i].y <= br.y){
            pROI.push_back(_p0[i]);
        }
    }
    return pROI;
}

void LucasMethod::drawBoundBox(std::vector<cv::Point2f> &_p0, cv::Mat &_frame){

    std::vector<cv::Point2f> hull;
    cv::convexHull(_p0, hull, true);
    cv::Rect bound = cv::boundingRect(hull);

    rectangle(_frame, bound, cv::Scalar(0, 255, 0), 4);
}