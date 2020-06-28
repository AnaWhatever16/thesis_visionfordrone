#include <autopilot/AutopilotGL.h>
#include <iostream>

bool AutopilotGL::init(std::map<std::string, std::string> _params) {
    int uav_id;

    /// Init ROS
    ros::param::param<int>("~uav_id", uav_id, 1);
    while (!ual_.isReady() && ros::ok()) {
        ROS_WARN("UAL %d not ready!", uav_id);
        sleep(1);
    }
    ROS_INFO("UAL %d ready!", uav_id);
    ros::spinOnce();
    
    /// Get image from drone
    ros::NodeHandle im;
    droneCam_= im.subscribe<sensor_msgs::Image>("/mbzirc_1/camera_0/image_raw", 10, &AutopilotGL::getCam,this);
    ros::spinOnce();
    
    //image = std::thread(&AutopilotGL::imageConvert,this);

    return true;
}
        
void AutopilotGL::targetPosition(const Eigen::Vector3f &_position, const float _yaw) {
    grvc::ual::Waypoint wp;
    wp.pose.position.x = _position(0);
    wp.pose.position.y = _position(1);
    wp.pose.position.z = _position(2);
    ual_.goToWaypoint(wp);

}

void AutopilotGL::targetSpeed(const Eigen::Vector3f &_speed, const float _yawRate) {
    geometry_msgs::TwistStamped vel;
    vel.twist.linear.x = _speed(0);
    vel.twist.linear.y = _speed(1);
    vel.twist.linear.z = _speed(2);

    vel.twist.angular.z = _yawRate;
    
    ual_.setVelocity(vel);
}

bool AutopilotGL::takeOff(const float _altitude) {
    ual_.takeOff(_altitude);
    ros::spinOnce();
}

void AutopilotGL::land() {
    ual_.land();
    ros::spinOnce();
}

bool AutopilotGL::isConnected() {
    return true;
}

void AutopilotGL::targetRpyThrust(const Eigen::Vector3f &_rpy, const float _thrust) {
    
}

void AutopilotGL::targetRpyAltitude(const Eigen::Vector3f &_rpy, const float _altitude) {
    
}

Eigen::Vector3f AutopilotGL::position() {
    Eigen::Vector3f pos;
    return pos; 
}

float AutopilotGL::yaw() {
    float ang=0;
    return ang;
}

Eigen::Matrix4f AutopilotGL::pose() {
    Eigen::Matrix4f pose;
    return pose;
}

void AutopilotGL:: getCam(const sensor_msgs::Image::ConstPtr& _msg){
    sensor_msgs::Image cam_;
    cam_ = *_msg;         
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(cam_, sensor_msgs::image_encodings::RGB8);
    droneImg_= cv_ptr->image;
    //cv::imshow("test", droneImg_);
} 