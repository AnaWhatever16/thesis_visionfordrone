#include <opencv2/core.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

/// Class used for rotation angle between two images.
/// @ingroup Ana_thesis
class AngleDetect{
    public:
        /// Constructor.
        /// \param _frame image to which compare.
        /// \param _vec all lines detected in the image. Each row represents a line defined as a point and a vector: [pointx, pointy, vx, vy]
        AngleDetect(cv::Mat &_frame, std::vector<cv::Vec4i> &_vec);

        ///Calculation of the matrix transform, T, and compare scores of all T's, so we can obtain the rotation angle.
        /// \param _vec all lines detected in the image. Each row represents a line defined as a point and a vector: [pointx, pointy, vx, vy]
        /// \param _frame image we want to compare.
        /// \return rotation angle in degrees.
        float detect(std::vector<cv::Vec4i> &_vec, cv::Mat &_frame);

    private:
        struct Match{
            int flag_eval;
            int id_line;
            float score;
        };

        cv::Mat initFrame_;
        std::vector<cv::Vec4i> vecInit_;
    
    private:
        //organizes the score calculation and selection. Selects min score for each ref line.
        float lineComparison(cv::Mat &_T, std::vector<cv::Vec4i> &_vecInit, std::vector<cv::Vec4i> &_vec, cv::Mat &_blend);
        //function to determine the equation of a line mx+n=y from a point and a vector
        cv::Vec3f lineCalc (cv::Vec4i _vec);
        //determines the score between two lines
        float scoreCalc(cv::Vec4i &_ref, cv::Vec4i &_obj);
        //goes through the score matrix and gives the total score (change matrix to struct)
        float scoreSelection(std::vector<Match> &_scores);
        float distanceCalc(cv::Vec4i &_ref, cv::Vec4i &_obj);
};