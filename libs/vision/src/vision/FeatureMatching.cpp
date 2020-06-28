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
//  strictly forbidden unless prior written permission is obtained from Vertical Engineering Solutions.
//
//---------------------------------------------------------------------------------------------------------------------
//
//  Maintainer: acasado@vengineerings.com
//
//---------------------------------------------------------------------------------------------------------------------

#include <vision/FeatureMatching.h>
#include <opencv2/tracking.hpp>
#include <opencv2/video.hpp>
#include <iostream>

FeatureMatching::FeatureMatching(std::string _argv){
    templ_ = cv::imread(_argv, cv::IMREAD_GRAYSCALE);
    detector_ = cv::ORB::create( minHessian );
    /// Detection of template's features
    detection(templ_, keypointsTempl_, descriptorsTempl_);

    //Matcher without filter
    matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    //Matcher with filter
    //matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
}

FeatureMatching::FeatureMatching(cv::Mat &_frame){
    templ_ = selectTemplate(_frame);
    detector_ = cv::ORB::create( minHessian );
    /// Detection of template's features
    detection(templ_, keypointsTempl_, descriptorsTempl_);

    //Matcher without filter
    matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    //Matcher with filter
    //matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

    imgCenter_ = cv::Point2f(_frame.cols/2 + templ_.cols, _frame.rows/2);
}

void FeatureMatching::method(cv::Mat &_input){
    /// Where keypoint detection of image occurs
    detection(_input, keypointsImg_, descriptorsImg_);
    /// Where matching between image and template occurs
    matching(_input);
}

void FeatureMatching::detection(cv::Mat &_input,std::vector<cv::KeyPoint> &_keypoints, cv::Mat &_descriptors)
{
    //-- Step 1: Detect the keypoints using ORB Detector
    detector_->detectAndCompute(_input, cv::noArray(), _keypoints, _descriptors);
}

void FeatureMatching::matching(cv::Mat &_img){
    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    // ORB uses Hamming distance to determine features
    if (descriptorsImg_.rows>12){
        std::vector< std::vector<cv::DMatch> > knn_matches;
        matcher_->knnMatch(descriptorsTempl_, descriptorsImg_, knn_matches, 2 );
        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.7f; //0.77f
        std::vector<cv::DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }
        //-- Draw matches
        cv::Mat img_matches;
        drawMatches( templ_, keypointsTempl_, _img, keypointsImg_, good_matches, img_matches, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Show detected matches
        if(!drawBoundBox(good_matches, img_matches)){
            ref_ = imgCenter_;
            cv::circle(img_matches, ref_, 1, cv::Scalar(0, 0, 0), 5);
        }

        cv::circle(img_matches, imgCenter_, 1, cv::Scalar(0, 0, 255), 5);

        imshow("Resultado", img_matches );
    }
    else{
        imshow("Resultado", _img);
    }
}
bool FeatureMatching::drawBoundBox(std::vector<cv::DMatch> &_good_matches, cv::Mat &_imgMatches){
    //-- Localize the object
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;

    for( int i = 0; i < _good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        scene.push_back( keypointsImg_[ _good_matches[i].trainIdx ].pt );
    }

    if (scene.size() > 0){
        std::vector<cv::Point2f> hull;
        cv::convexHull(scene, hull, true);
        bound_ = cv::boundingRect(hull);
        cv::Point2f ctl = bound_.tl();
        cv::Point2f cbr = bound_.br();
        cv::Size siz = bound_.size();
        cv::Point2f cbl = ctl + cv::Point2f(0, siz.height); 
        cv::Point2f ctr = ctl + cv::Point2f(siz.width, 0);

        std::vector<cv::Point2f> scene_corners(4);
        scene_corners[0]=ctl; scene_corners[1]= ctr;
        scene_corners[2]= cbr; scene_corners[3]= cbl;

        rectangle( _imgMatches, ctl + cv::Point2f( templ_.cols, 0), cbr + cv::Point2f( templ_.cols, 0), cv::Scalar(0, 255, 0), 4);

        //Calculate centroid
        cv::Moments mu;
        mu = moments(scene_corners);
        cv::Point2f c(mu.m10/mu.m00, mu.m01/mu.m00);

        ref_= c + cv::Point2f( templ_.cols, 0);
        cv::circle(_imgMatches, ref_, 1, cv::Scalar(0, 0, 0), 5);

        return true;
    }
    else{
        return false;
    }

}

cv::Mat FeatureMatching::selectTemplate(cv::Mat &_frame){
    cv::Rect2d roi = cv::selectROI("ROI", _frame, true, false);

    while(roi.width==0 || roi.height==0){
        std::cout << "Try again" << std::endl;
        roi = selectROI("ROI",_frame, false);
    }

    cv::destroyWindow("ROI");

    cv::Mat newTemp = cv::Mat(_frame, roi);

    return newTemp;
}