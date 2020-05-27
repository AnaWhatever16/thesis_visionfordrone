#include <opencv2/opencv.hpp> // OpenCV libraries

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>

class RealSenseCamera{
    public:
        enum class RS_TYPE {D435, T265};

        RealSenseCamera(RS_TYPE _type, const Eigen::Matrix4f &_body2cam = Eigen::Matrix4f::Identity());

        RealSenseCamera &operator>> (cv::Mat &_giveframe); //image
        RealSenseCamera &operator>> (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_givecloud); //pointcloud
        RealSenseCamera &operator>> (Eigen::Matrix4f &_pose); //pose matrix

        void reset();

        /// Method available for T265 to get if estimation is confident.
        bool isConfident() const {return lastPoseConfident_;}

    private:
        cv::Mat queryImage();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr queryPointCloud();
        Eigen::Matrix4f queryPose();
        
    private:
        cv::Mat lastImage_;
        rs2::pipeline pipe_;
        rs2::config cfg_;

        Eigen::Matrix4f body2cam_;
        bool lastPoseConfident_ = false;
};