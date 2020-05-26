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

#include <opencv2/core.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <aerox_vision/algorithms/AngleDetect.h>

namespace aerox{
    /// Class for implementation of the algorithm of Kernel Correlation Filter. 
    /// @ingroup aerox_vision
    /// This class uses class AngleDetect to calculate the angle of rotation of the object that is going to be tracked. 
    class KCFTracker{
        public: 
            /// Constructor. 
            /// Initializes tracker and is where you will choose what part of the image you want to track.
            /// It will draw the center of the image selected.
            /// \param _frame first image where you will choose what to track.
            KCFTracker(cv::Mat &_frame); 

            /// Function which checks where the image selected previously is and redraws where the center should be.
            /// \param _frame image to which we have to compare the previos one (update).
            /// \return true if update was successful, false if not.
            bool update(cv::Mat &_frame);

            /// Function to get position of the center of the tracked object.
            /// \return center of object tracked.
            cv::Point2f getRef(){return ref_;} // where the center of the object selected is

            /// Function to get position of the center of the whole image.
            /// \return center of image.
            cv::Point2f getPos(){return imgCenter_;} // the center of the image

            /// Function to get rotation angle of the tracked object with respect with the beginning.
            /// \return angle in degrees. 
            double getAngle(){return anglePos_;}

        private:
            cv::Ptr<cv::TrackerKCF> tracker_;
            cv::Rect2d roi_;

            cv::Point2f ref_;
            std::vector<cv::Vec4i> vecInit_;
            cv::Mat frameInit_;

            cv::Point2f imgCenter_;
            float anglePos_;
            cv::Mat frameBinary_;

            AngleDetect *angle;

        private:
            bool selectRef(cv::Mat &_frame);
            std::vector<cv::Point2f> calcRoi();
            std::vector<cv::Vec4i> computeCandidateLines(cv::Mat &_frame);
            std::vector<cv::Vec4i> defineLines(std::vector<cv::Vec4i> &_lines);
    };
}