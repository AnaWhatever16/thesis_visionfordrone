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

#include <opencv2/core.hpp> // OpenCV libraries

namespace aerox{
    /// Class for implementation of the algorithm of Template Matching using threads to improve computation times.
    /// @ingroup aerox_vision
    class TemplateMatchThread{
        public:
            /// Constructor. It takes the template given in command line
            /// and resizes it so it can detect the template even if 
            /// the distance to the object changes. 
            /// \param _argv path to image
            TemplateMatchThread(std::string _argv);

            /// Constructor. It takes the template given as a image variable (cv::Mat)
            /// and resizes it so it can detect the template even if 
            /// the distance to the object changes. 
            /// \param _argv path to image
            TemplateMatchThread(cv::Mat &_frame);

            /// Algorithm of Template Matching. This class creates a window with the image processed with 
            /// a green rectangle showing where the template image should be. 
            /// \param _input image we want to process
            /// \param _matchMethod score calculation method. Methods that can be used can be found <a href=https://docs.opencv.org/master/df/dfb/group__imgproc__object.html#ga3a7850640f1fe1f58fe91a2d7583695d>here</a>.
            double matchThread(cv::Mat &_input, int _matchMethod);

        private:
            cv::Mat templ_;
            std::vector<cv::Mat> templResize_;
    };
}