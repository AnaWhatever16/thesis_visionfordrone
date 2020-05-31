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

        
    private:
        std::vector<cv::Scalar> colors_;
        cv::Mat oldFrame_, oldGray_;
        std::vector<cv::Point2f> p0_;
        cv::Mat mask;
        cv::TermCriteria criteria;

    private:
        std::vector<cv::Point2f> goodTrackingFeatures(std::vector<cv::Point2f> &_p0);
        void drawBoundBox(std::vector<cv::Point2f> &_p0, cv::Mat &_frame);
};