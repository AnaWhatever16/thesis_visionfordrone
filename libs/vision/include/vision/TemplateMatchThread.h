#include <opencv2/core.hpp> // OpenCV libraries

/// Class for implementation of the algorithm of Template Matching using threads to improve computation times.
/// @ingroup Ana_thesis
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

        /// Function to get point where the center of the template is
        /// \return center of template
        cv::Point2f getObjectSelected(){return templCenter_;}

        /// Function to get point where the center of the image is
        /// \return center of image
        cv::Point2f getCenterImage(){return imgCenter_;}

    private:
        cv::Mat templ_;
        std::vector<cv::Mat> templResize_;

        cv::Point2f imgCenter_;
        cv::Point2f templCenter_;
    
    private: 
        cv::Mat selectTemplate(cv::Mat &_frame);
};