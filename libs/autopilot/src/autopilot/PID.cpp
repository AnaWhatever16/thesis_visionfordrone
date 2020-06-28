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
//  strictly forbidden unless prior written permission is obtained from Adobe Systems Incorporated.
//
//---------------------------------------------------------------------------------------------------------------------
//
//  Maintainer: pramon@vengineerings.com
//
//---------------------------------------------------------------------------------------------------------------------

#include <autopilot/PID.h>
#include <algorithm>
#include <thread>
#include <chrono>
#include <iostream>

namespace aerox{

    PID::PID(float _kp, float _ki, float _kd, float _minSat, float _maxSat, float _minWind, float _maxWind) {
        mKp = _kp;
        mKi = _ki;
        mKd = _kd;
        mMinSat = _minSat;
        mMaxSat = _maxSat;
        mWindupMin = _minWind;
        mWindupMax = _maxWind;
		std::cout << "PID PARAMS " << mKp << ", "<< mKi << ", "<< mKd << ", "<< mMinSat << ", "<< mMaxSat << ", "<< mWindupMin << ", "<< mWindupMax << "\n";
    }

    float PID::update(float _val, float _incT) {
        float dt = _incT; // 666 input arg?

        float err = mReference - _val;
        mAccumErr += err * dt;
        // Apply anti wind-up 777 Analyze other options
        mAccumErr = std::min(std::max(mAccumErr, mWindupMin), mWindupMax);

        // Compute PID
        mLastResult = mKp * err + mKi * mAccumErr + mKd * (err - mLastError) / dt;
        mLastError = err;

        // Saturate signal
        mLastResult = std::min(std::max(mLastResult, mMinSat), mMaxSat);
        mLastResult *= mBouncingFactor;
        mBouncingFactor *= 2.0;
        mBouncingFactor = mBouncingFactor > 1.0 ? 1.0 : mBouncingFactor;
        return mLastResult;
    }

    //------------------------------------------------------------------------------------------------------------------------------
    void PID::enableRosPublisher(std::string _topic){
        #ifdef HAS_ROS
            mRosPubParams = nh_.advertise<std_msgs::Float32MultiArray>(_topic, 1);
            run_ = false;
            if(mParamPubThread.joinable())
                mParamPubThread.join();
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            run_ = true;
            mParamPubThread = std::thread([&](){
                while(run_){
                    std_msgs::Float32MultiArray data;
                    data.data = {mKp, mKi, mKd, mMaxSat, mWindupMax};
                    mRosPubParams.publish(data);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            });
        #endif
    }

    //------------------------------------------------------------------------------------------------------------------------------
    void PID::enableRosSubscriber(std::string _topic){
        #ifdef HAS_ROS
            mRosSubParams = nh_.subscribe<std_msgs::Float32MultiArray>(_topic, 1, &PID::rosSubCallback, this);
        #endif
    }

    //------------------------------------------------------------------------------------------------------------------------------
    void PID::enableFastcomPublisher(int _port){
        #ifdef HAS_FASTCOM
            if(mFastcomPubParams) delete mFastcomPubParams;
            mFastcomPubParams = new fastcom::Publisher<PIDParams>(_port);
            run_ = false;
            if(mParamPubThread.joinable())
                mParamPubThread.join();
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            run_ = true;
            mParamPubThread = std::thread([&](){
                while(run_){
                    PIDParams params = {mKp, mKi, mKd, mMaxSat, mWindupMax};
                    mFastcomPubParams->publish(params);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            });
        #endif
    }

    //------------------------------------------------------------------------------------------------------------------------------
    void PID::enableFastcomSubscriber(int _port){
        #ifdef HAS_FASTCOM
            if(mFastcomSubParams) delete mFastcomSubParams;
            mFastcomSubParams  = new fastcom::Subscriber<PIDParams>("10.0.0.111", _port);
    std::cout << "trying to subscribe" << std::endl;
            mPort = _port;
            auto callback = [this](const PIDParams &_params){
    ignoreCounter++;
    if(ignoreCounter < 10)
        return;
                    mKp = _params.kp;
                    mKi = _params.ki;
                    mKd = _params.kd;
                    mMinSat = -_params.sat;
                    mMaxSat =  _params.sat;
                    mWindupMin = -_params.wind;
                    mWindupMax =  _params.wind;
                };
            mFastcomSubParams->attachCallback(callback);
        #endif
    }


    #ifdef HAS_ROS
    void PID::rosSubCallback(const std_msgs::Float32MultiArray::ConstPtr &_msg){
        std::cout << "Received message" << std::endl;
        this->mKp = _msg->data[0];
        this->mKi = _msg->data[1];
        this->mKd = _msg->data[2];
        this->mMinSat = -_msg->data[3];
        this->mMaxSat = _msg->data[3];
        this->mWindupMin = -_msg->data[4];
        this->mWindupMax = _msg->data[4];
    }
    #endif

}

