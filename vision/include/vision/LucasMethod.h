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

namespace aerox{
    /// Class for implementation of the algorithm of Optical Flow: Lucas.
    /// @ingroup aerox_vision
    class LucasMethod{
        public:

            /// Constructor. 
            /// Analyzes which features to track through the rest of the execution of the program.
            /// \param _input first frame to analyze.
            LucasMethod(cv::Mat &_input);

            /// Draws optical flow of initial features.
            /// \param _input frame to analyze.
            void method(cv::Mat &_input);
            
        private:
            std::vector<cv::Scalar> colors;
            cv::Mat old_frame, old_gray;
            std::vector<cv::Point2f> p0, p1;
            cv::Mat mask;
            cv::TermCriteria criteria;
    };
}