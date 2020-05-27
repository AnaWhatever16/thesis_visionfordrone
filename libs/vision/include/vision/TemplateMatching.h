#include <opencv2/core.hpp> // OpenCV libraries

/// Class for implementation of the algorithm of Template Matching.
/// @ingroup Ana_thesis
class TemplateMatching{
    public:
        /// Constructor. It takes the template given in command line
        /// and resizes it so it can detect the template even if 
        /// the distance to the object changes. 
        /// \param _argv path to image
        TemplateMatching(std::string _argv);

        /// Algorithm of Template Matching. This class creates a window with the image processed with 
        /// a green rectangle showing where the template image should be. 
        /// \param _input image we want to process
        /// \param _matchMethod score calculation method. Methods that can be used can be found <a href=https://docs.opencv.org/master/df/dfb/group__imgproc__object.html#ga3a7850640f1fe1f58fe91a2d7583695d>here</a>.
        void matchingMethod(cv::Mat &_input, int _matchMethod);
        
    private:
        cv::Mat templ_;
        std::vector<cv::Mat> templResize_;
};