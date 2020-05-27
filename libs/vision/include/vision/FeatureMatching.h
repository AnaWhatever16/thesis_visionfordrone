#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc.hpp>

/// Class for implementation of the algorithm of Feature Matching.
/// @ingroup Ana_thesis
class FeatureMatching{
    public: 
        /// Constructor. It takes the template given in command line,
        /// creates the detector and matcher
        /// and analyzes template features (creates its keypoints) 
        /// \param _argv path to image
        FeatureMatching(std::string _argv);

        /// Function where detection of the image keypoints are detected 
        /// and matches with template are drawn. 
        /// The result is a bounding box where the template should be. 
        void method(cv::Mat &_input);

        /// Function to get point where the center of the template is
        /// \return center of template
        cv::Point2f getRef(){return ref_;}

        /// Function to get point where the center of the image is
        /// \return center of image
        cv::Point2f getPos(){return imgCenter_;}

    private:
        cv::Mat templ_;
        int minHessian = 1000; //400 realsense 1000 drone
        std::vector<cv::KeyPoint> keypointsTempl_;
        cv::Mat descriptorsTempl_;

        std::vector<cv::KeyPoint> keypointsImg_;
        cv::Mat descriptorsImg_;

        cv::Ptr<cv::ORB> detector;
        cv::Ptr<cv::DescriptorMatcher> matcher;

        cv::Point2f ref_;
        cv::Point2f imgCenter_;
    
    private:
        void detection(cv::Mat &_input, std::vector<cv::KeyPoint> &_keypoints, cv::Mat &_descriptors);
        void matching(cv::Mat &_img);
        void drawBoundBox(std::vector<cv::DMatch> &_good_matches, cv::Mat &_imgMatches);
    
};