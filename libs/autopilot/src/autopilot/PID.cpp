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
//  Maintainer: pramon@vengineerings.com
//
//---------------------------------------------------------------------------------------------------------------------

#include <autopilot/PID.h>
#include <algorithm>
#include <thread>
#include <chrono>
#include <iostream>
#include <cmath>
#include <Eigen/Eigen>

#include <cmath>

namespace aerox{

    float PID::EuclideanDistance(float _a, float _b) { return _b - _a; };
    float PID::AngularDistance(float _a, float _b){
        Eigen::Vector3f v1 = {cos(_a), sin(_a), 0};
        Eigen::Vector3f v2 = {cos(_b), sin(_b), 0};
        auto v3 = v1.cross(v2);
        float magn = acos(v1.dot(v2));
        float angle = v3[2];
        // std::cout << "Angle (" << _a << "\t," << _b <<")\t="<<angle << std::endl;
    };


    PID::PID(float _kp, float _ki, float _kd, float _minSat, float _maxSat) {
        kp_ = _kp;
        ki_ = _ki;
        kd_ = _kd;
        minSat_ = _minSat;
        maxSat_ = _maxSat;
        reference_ = 0;
        updateFn_ = std::bind(&PID::updateAWU_None, this, std::placeholders::_1, std::placeholders::_2);
		// std::cout << "PID PARAMS " << kp_ << ", "<< ki_ << ", "<< kd_ << ", "<< minSat_ << ", "<< maxSat_ << ", "<< minWindup_ << ", "<< maxWindup_ << "\n";
    }

    void PID::setAntiWindup(AntiWindupMethod _antiWindup, std::vector<float> _params){
        switch (_antiWindup) {
        case AntiWindupMethod::None:
            updateFn_ = std::bind(&PID::updateAWU_None,             this, std::placeholders::_1, std::placeholders::_2);
            break;
        case AntiWindupMethod::Saturation:
            assert(_params.size() == 2);
            updateFn_ = std::bind(&PID::updateAWU_Saturation,       this, std::placeholders::_1, std::placeholders::_2);
            minWindup_ = _params[0];
            maxWindup_ = _params[1];
            break;
        case AntiWindupMethod::BackCalculation:
            assert(_params.size() == 1);
            updateFn_ = std::bind(&PID::updateAWU_BackCalculation,  this, std::placeholders::_1, std::placeholders::_2);
            backCalculationCte_ = _params[0];
            break;
        case AntiWindupMethod::Clamping:
            updateFn_ = std::bind(&PID::updateAWU_Clamping,         this, std::placeholders::_1, std::placeholders::_2);
            break;
        
        }
    }


    void PID::reference(float _ref, bool _reset) { 
        reference_ = _ref; 
        if(_reset){
            accumErr_ = 0; 
            lastError_ = 0; 
            lastResult_ = 0; 
            bouncingFactor_ = 0.1;
        }
    }


    float PID::update(float _val, float _incT) {
        return updateFn_(_val, _incT);
    }

    //----------------------------------------------------
    float PID::updateAWU_None(float _val, float _incT){
        float dt = _incT; // 666 input arg?

        float err = distanceFn_(_val, reference_);//reference_ - _val;
        accumErr_ += err * dt;


        // Compute PID
        float unsaturated =     kp_ * err + 
                                ki_ * accumErr_ + 
                                kd_ * (err - lastError_) / dt;
        lastError_ = err;

        // Saturate signal
        float saturated = std::min(std::max(unsaturated, minSat_), maxSat_);
        lastResult_ = saturated * bouncingFactor_;
        bouncingFactor_ *= 2.0;
        bouncingFactor_ = bouncingFactor_ > 1.0 ? 1.0 : bouncingFactor_;
        return lastResult_;
    }

    float PID::updateAWU_Saturation(float _val, float _incT){
        float dt = _incT; // 666 input arg?

        float err = distanceFn_(_val, reference_);//reference_ - _val;
        accumErr_ += err * dt;
        accumErr_ = std::min(std::max(accumErr_, minWindup_), maxWindup_);

        // Compute PID
        float unsaturated =     kp_ * err + 
                                ki_ * accumErr_ + 
                                kd_ * (err - lastError_) / dt;

        lastError_ = err;

        // Saturate signal
        float saturated = std::min(std::max(unsaturated, minSat_), maxSat_);
        lastResult_ = saturated;
        // lastResult_ = saturated * bouncingFactor_;
        // bouncingFactor_ *= 2.0;
        // bouncingFactor_ = bouncingFactor_ > 1.0 ? 1.0 : bouncingFactor_;
        return lastResult_;
    }

    float PID::updateAWU_BackCalculation(float _val, float _incT){
        float dt = _incT; // 666 input arg?

        float err = distanceFn_(_val, reference_);//reference_ - _val;
        accumErr_ += err * dt +  backCalculationCte_*lastBackCalculation_*dt;

        // Compute PID
        float unsaturated =     kp_ * err + 
                                ki_ * accumErr_ + backCalculationCte_*lastBackCalculation_ + 
                                kd_ * (err - lastError_) / dt;
        lastError_ = err;

        // Saturate signal
        float saturated = std::min(std::max(unsaturated, minSat_), maxSat_);
        lastBackCalculation_ = saturated - unsaturated;
        lastResult_ = saturated;
        // lastResult_ = saturated * bouncingFactor_;
        // bouncingFactor_ *= 2.0;
        // bouncingFactor_ = bouncingFactor_ > 1.0 ? 1.0 : bouncingFactor_;
        return lastResult_;

    }

    float PID::updateAWU_Clamping(float _val, float _incT){
        float dt = _incT; // 666 input arg?

        float err = distanceFn_(_val, reference_);//reference_ - _val;
        accumErr_ += err * dt;

        // Clamping
        bool isSaturated = (lastResult_ <= minSat_ || lastResult_ >= maxSat_);
        bool sameSign = std::signbit(err) == std::signbit(lastResult_);
        clampFactor_ = (!isSaturated && sameSign);

        // Compute PID
        float unsaturated =     kp_ * err + 
                                clampFactor_* ki_ * accumErr_ + 
                                kd_ * (err - lastError_) / dt;
        lastError_ = err;

        // Saturate signal
        float saturated = std::min(std::max(unsaturated, minSat_), maxSat_);
        lastResult_ = saturated;
        //lastResult_ = saturated * bouncingFactor_;
        //bouncingFactor_ *= 2.0;
        //bouncingFactor_ = std::min(bouncingFactor_, 1.0);
        return lastResult_;
    }


}

