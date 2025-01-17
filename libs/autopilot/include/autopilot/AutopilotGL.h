#include <ros/ros.h>

#include <uav_abstraction_layer/ual.h>
#include <uav_abstraction_layer/backend.h>
#include <ual_backend_gazebo_light/ual_backend_gazebo_light.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>
#include <Eigen/Eigen>

///Class that abstract connection with UAL autopilot in simulation with Gazebo 
/// @ingroup Ana_thesis
class AutopilotGL {
public:

    ///Constructor. It is used to initialize the UAL in Gazebo
    AutopilotGL() : ual_(new grvc::ual::BackendGazeboLight()){ }    

    ///Initializes connection with ROS and connects simulation image with subscriber
    /// \param _params not used
    /// \return true when ROS is setup        
    bool init(std::map<std::string, std::string> _params);
    
    /// Set a target position for the  UAV to hover on.
    /// \param _position: Target position for the UAV.
    /// \param _yaw: target orientation of the UAV.
    /// Not implemented
    void targetPosition(const Eigen::Vector3f &_position, const float _yaw);

    /// Set a target speed for the robot
    /// \param _speed: Target peed for the UAV
    /// \param _yaw: target orientation speed of the UAV.
    void targetSpeed(const Eigen::Vector3f &_speed, const float _yawRate);

    /// Send take off command to autopilot. It is not granted that all the autopilots go to the given altitude when
    /// taking off. 
    /// \param _altitude: target altitude for the takeoff.
    bool takeOff(const float _altitude); 

    /// Send a land command to autopilot.
    void land();

    /// Method to check if the application is connected to the autopilot.
    /// \return true if connected false if not.
    bool isConnected();

    /// Set a target rool, pitch and yaw for the autopilot. For the altitude, thrust is used.
    /// Please see specific implementation of Autopilot before using this method.
    /// \param _rpy: Target peed for the UAV.
    /// \param _thrust: target thrust for autopilot.
    /// Not implemented
    void targetRpyThrust(const Eigen::Vector3f &_rpy, const float _thrust);

    /// Set a target rool, pitch and yaw for the autopilot. This method intend to hold the target  directly.
    /// Please see specific implementation of Autopilot before using this method. This method is not granted to be
    /// implemented in all autopilots.
    /// \param _rpy: Target peed for the UAV.
    /// \param _alatitude: target thrust for autopilot.
    /// Not implemented
    void targetRpyAltitude(const Eigen::Vector3f &_rpy, const float _altitude);

    /// Get position estimate from autopilot.
    /// Not implemented
    Eigen::Vector3f position(); 

    /// Get yaw estimate from autopilot.
    /// Not implemented
    float yaw();

    /// Get pose estimate from autopilot.
    /// Not implemented 
    Eigen::Matrix4f pose();       
    
    /// Image from camera under the drone
    cv::Mat droneImg_;

private:
    grvc::ual::UAL ual_;
    ros::Subscriber droneCam_;
    void getCam(const sensor_msgs::Image::ConstPtr &_msg);
}; 