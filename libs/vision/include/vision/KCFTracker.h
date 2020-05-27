#include <opencv2/core.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vision/AngleDetect.h>

/// Class for implementation of the algorithm of Kernel Correlation Filter. 
/// @ingroup Ana_thesis
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