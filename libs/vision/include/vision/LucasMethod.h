#include <opencv2/core.hpp>

/// Class for implementation of the algorithm of Optical Flow: Lucas.
/// @ingroup Ana_thesis
class LucasMethod{
    public:

        /// Constructor. 
        /// Analyzes which features to track through the rest of the execution of the program.
        /// \param _input first frame to analyze.
        LucasMethod(cv::Mat &_input);

        /// Draws optical flow of initial features.
        /// \param _input frame to analyze.
        void method(cv::Mat &_input);

        /// Function to get point where the center of the template is
        /// \return center of template
        cv::Point2f getObjectSelected(){return objectCenter_;}

        /// Function to get point where the center of the image is
        /// \return center of image
        cv::Point2f getCenterImage(){return imgCenter_;}

        
    private:
        std::vector<cv::Scalar> colors_;
        cv::Mat oldFrame_, oldGray_;
        std::vector<cv::Point2f> p0_;
        cv::Mat mask_;
        cv::TermCriteria criteria_;

        cv::Point2d objectCenter_;
        cv::Point2d imgCenter_;

    private:
        std::vector<cv::Point2f> goodTrackingFeatures(std::vector<cv::Point2f> &_p0);
        bool drawBoundBox(std::vector<cv::Point2f> &_p0, cv::Mat &_frame);
};