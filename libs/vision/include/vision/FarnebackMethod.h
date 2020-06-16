#include <opencv2/core.hpp>

/// Class for implementation of the algorithm of Optical Flow: Farneback.
/// @ingroup Ana_thesis
class FarnebackMethod{
    public:
        /// Constructor. 
        /// Analyzes which features to track first.
        /// \param _input first frame to analyze.
        FarnebackMethod(cv::Mat &_input);

        /// Draws optical flow of features found in previous frame.
        /// \param _input frame to analyze.
        void method(cv::Mat &_input);
    private:
        cv::Mat prvs_;
};