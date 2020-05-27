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
        std::vector<cv::Scalar> colors;
        cv::Mat old_frame, old_gray;
        std::vector<cv::Point2f> p0, p1;
        cv::Mat mask;
        cv::TermCriteria criteria;
};