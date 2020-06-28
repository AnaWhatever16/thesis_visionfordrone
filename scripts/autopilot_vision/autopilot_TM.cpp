#include <ros/ros.h>
#include <autopilot/AutopilotGL.h>
#include <autopilot/PID.h>
#include <autopilot/RateCounter.h>
#include <vision/TemplateMatchThread.h>

#include <iostream>
#include <unistd.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main (int argc, char **_argv){
    ros::init(argc, _argv, "AutopilotGL");
    AutopilotGL drone;

    aerox::PID controlX(0.006,0.4,0,-std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                                       std::numeric_limits<float>::min(),-std::numeric_limits<float>::min());

    aerox::PID controlY(0.006,0.4,0,-std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 
                                       std::numeric_limits<float>::min(),-std::numeric_limits<float>::min());

    //aerox::PID controlYaw(0,0,0);

    float dt = 0.01;

    std::map<std::string, std::string> m;
    drone.init(m);

    drone.takeOff(5);
    
    Eigen::Vector3f vel(0,0,0); 
    float velx = 0, vely = 0;
    const float yaw_rate = 0;   

    while((char)27!=cv::waitKey(3)){
        cv::imshow("takeOff", drone.droneImg_); 
    }

    cv::destroyWindow("takeOff");

    TemplateMatchThread tracker(drone.droneImg_);

    cv::Point2f ref;
    cv::Point2f pos;

    ref=tracker.getCenterImage();

    controlX.reference(ref.y);
    controlY.reference(ref.x);
    
    aerox::RateCounter counter;
    //int match_method=cv::TM_SQDIFF
    //int match_method=cv::TM_SQDIFF_NORMED;
    //int match_method=cv::TM_CCORR;
    //int match_method=cv::TM_CCORR_NORMED
    //int match_method=cv::TM_CCOEFF;
    int match_method=cv::TM_CCOEFF_NORMED;

    while((char)27!=cv::waitKey(1)){
        //the axis of the drone are not the same as the axis of the image
        tracker.matchThread(drone.droneImg_, match_method);

        pos=tracker.getObjectSelected();

        //controlYaw.reference(yaw_rate);

        counter.update();
        dt=1/counter.hz();

        velx = controlX.update(pos.y, dt);
        vely = controlY.update(pos.x, dt);

        // std::cout<<"INPUTX= "<<ref.y<<" pixel || ERROR= "<<ref.y-pos.y<<" pixels || CONTROL= "<<velx<<" m/s"<<std::endl<<std::flush;
        // std::cout<<"INPUTY= "<<ref.x<<" pixel || ERROR= "<<ref.x-pos.x<<" pixels || CONTROL= "<<vely<<" m/s"<<std::endl<<std::flush;
        
        vel(0) = velx;
        vel(1) = vely;
        drone.targetSpeed(vel, yaw_rate);

        //cv::waitKey(3);
    }
    // std::cout << drone.pose() << std::endl;
    cv::destroyAllWindows();
    drone.land();

    return 0;
}