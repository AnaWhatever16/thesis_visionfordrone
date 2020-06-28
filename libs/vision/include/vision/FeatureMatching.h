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

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc.hpp>

/// Class for implementation of the algorithm of Feature Matching.
/// @ingroup aerox_vision
class FeatureMatching{
    public: 
        /// Constructor. It takes the template given in command line,
        /// creates the detector and matcher
        /// and analyzes template features (creates its keypoints) 
        /// \param _argv path to image
        FeatureMatching(std::string _argv);

        /// Constructor. You choose the template from an image,
        /// creates the detector and matcher
        /// and analyzes template features (creates its keypoints) 
        /// \param _argv path to image
        FeatureMatching(cv::Mat &_frame);

        /// Function where detection of the image keypoints are detected 
        /// and matches with template are drawn. 
        /// The result is a bounding box where the template should be. 
        void method(cv::Mat &_input);

        /// Function to get point where the center of the template is
        /// \return center of template
        cv::Point2f getObjectSelected(){return ref_;}

        /// Function to get point where the center of the image is
        /// \return center of image
        cv::Point2f getCenterImage(){return imgCenter_;}

        cv::Rect getROI(){return bound_;}

    private:
        cv::Mat templ_;
        int minHessian = 5000; //400 realsense
        std::vector<cv::KeyPoint> keypointsTempl_;
        cv::Mat descriptorsTempl_;

        std::vector<cv::KeyPoint> keypointsImg_;
        cv::Mat descriptorsImg_;

        cv::Ptr<cv::ORB> detector_;
        cv::Ptr<cv::DescriptorMatcher> matcher_;

        cv::Point2f ref_;
        cv::Point2f imgCenter_;
    
    private:
        void detection(cv::Mat &_input, std::vector<cv::KeyPoint> &_keypoints, cv::Mat &_descriptors);
        void matching(cv::Mat &_img);
        void drawBoundBox(std::vector<cv::DMatch> &_good_matches, cv::Mat &_imgMatches);
        cv::Mat selectTemplate(cv::Mat &_frame);
        cv::Rect bound_;
    
};