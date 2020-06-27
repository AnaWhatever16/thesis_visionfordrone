//---------------------------------------------------------------------------------------------------------------------
//  Vertical Engineering Solutions
//---------------------------------------------------------------------------------------------------------------------
// 
//  Copyright 2020 Vertical Engineering Solutions  - All Rights Reserved
// 
//  Unauthorized copying of this file, via any medium is strictly prohibited Proprietary and confidential.
// 
//  All information contained herein is, and remains the property of Vertical Engineering Solutions.  The 
//  intellectual and technical concepts contained herein are proprietary to Vertical Engineering Solutions 
//  and its suppliers and may be covered by UE and Foreign Patents, patents in process, and are protected 
//  by trade secret or copyright law. Dissemination of this information or reproduction of this material is 
//  strictly forbidden unless prior written permission is obtained from Vertical Engineering Solutions.
//
//---------------------------------------------------------------------------------------------------------------------
//
//  Maintainer: acasado@vengineerings.com
//
//---------------------------------------------------------------------------------------------------------------------

#include <ros/ros.h>
#include <autopilot/AutopilotGL.h>
#include <autopilot/PID.h>
#include <autopilot/RateCounter.h>
#include <vision/KCFTracker.h>

#include <iostream>
#include <unistd.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main (int argc, char **_argv){
    ros::init(argc, _argv, "AutopilotGL");
    AutopilotGL drone;

    aerox::PID controlX(0.023,2,0.003,-std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    controlX.setAntiWindup( aerox::PID::AntiWindupMethod::Clamping, 
                            {std::numeric_limits<float>::min(),-std::numeric_limits<float>::min()});

    aerox::PID controlY(0.023,2,0.003,-std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    controlY.setAntiWindup( aerox::PID::AntiWindupMethod::Clamping, 
                            {std::numeric_limits<float>::min(),-std::numeric_limits<float>::min()});

    aerox::PID controlYaw(0,0,0);

    float dt = 0.01;

    std::map<std::string, std::string> m;
    drone.init(m);

    drone.takeOff(5);
    
    Eigen::Vector3f vel(0,0,0); 
    float velx = 0, vely = 0;
    const float yaw_rate = 0;   

    while((char)27!=cv::waitKey(1)){
        cv::imshow("takeOff", drone.droneImg_); 
    }

    cv::destroyWindow("takeOff");

    KCFTracker tracker(drone.droneImg_, 0);

    cv::Point2f ref;
    cv::Point2f pos;

    int flag = 0;
    aerox::RateCounter counter;

    while((char)27!=cv::waitKey(1)){
        //the axis of the drone are not the same as the axis of the image
        if(!tracker.update(drone.droneImg_)){
            std::cout << "The target has been lost..."<< std::endl;
            break;
        }

        if (!flag){
            ref=tracker.getCenterImage();
            flag=1;
        } 

        pos=tracker.getObjectSelected();

        controlX.reference(ref.y); // you could do it only once...
        controlY.reference(ref.x);
        //controlYaw.reference(yaw_rate);

        counter.update();
        dt=1/counter.hz();

        velx = controlX.update(pos.y, dt);
        vely = controlY.update(pos.x, dt);

        //std::cout<<"INPUTX= "<<ref.y<<" pixel || ERROR= "<<ref.y-pos.y<<" pixels || CONTROL= "<<velx<<" m/s"<<std::endl<<std::flush;
        //std::cout<<"INPUTY= "<<ref.x<<" pixel || ERROR= "<<ref.x-pos.x<<" pixels || CONTROL= "<<vely<<" m/s"<<std::endl<<std::flush;
        
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