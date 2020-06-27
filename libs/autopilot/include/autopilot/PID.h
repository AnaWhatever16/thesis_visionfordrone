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


#include <limits>
#include <thread>
#include <functional>

#include <cmath>
#include <vector>

namespace aerox{

    class PID {
    public:
        struct PIDParams{
            float kp, ki, kd, sat, wind;
        };

        enum class AntiWindupMethod { None, Saturation, BackCalculation, Clamping};

        PID(float _kp, float _ki, float _kd,
            float _minSat = -99999,
            float _maxSat = 99999 );
        
        void setAntiWindup(AntiWindupMethod _antiWindup, std::vector<float> _params = {});

        float update(float _val, float _incT);
        
        void distanceFunction(std::function<float(float,float)> _fn) { distanceFn_ = _fn; };

        float reference() { return reference_; }
        void reference(float _ref, bool _reset = true);

        float kp() const { return kp_; }
        float ki() const { return ki_; }
        float kd() const { return kd_; }
    
        void kp(float _kp) { kp_ = _kp; }
        void ki(float _ki) { 
            // Scale accumulated error to prevent abrupt changes in integral component of control signal
            accumErr_ *=ki_/_ki;    
            ki_ = _ki; 
        }
        void kd(float _kd) { kd_ = _kd; }
    
        void setSaturations(float _min, float _max) { minSat_ = _min; maxSat_ = _max; }
        void getSaturations(float &_min, float &_max) { _min = minSat_; _max = maxSat_; }
    
        void setWindupTerms(float _min, float _max) { minWindup_ = _min; maxWindup_ = _max; }
        void getWindupTerms(float &_min, float &_max) { _min = minWindup_; _max = maxWindup_; }

    public:
        static float EuclideanDistance(float _a, float _b);
        static float AngularDistance(float _a, float _b);

    private:    // AntiWindUp implementations
        float updateAWU_None(float _val, float _incT);
        float updateAWU_Saturation(float _val, float _incT);
        float updateAWU_BackCalculation(float _val, float _incT);
        float updateAWU_Clamping(float _val, float _incT);

    private:
        float reference_;
        float kp_, ki_, kd_;
        float minSat_, maxSat_;
        float lastResult_ = 0, lastError_ = 0, accumErr_ = 0;
        double bouncingFactor_ = 0.1;

        // Params related with antiwindups
        // Saturation
        float minWindup_, maxWindup_;
        // BackCalculation
        float lastBackCalculation_ = 0;
        float backCalculationCte_ = 1;
        // Clamping
        bool clampFactor_ = 1;


        std::function<float(float, float)> distanceFn_ = EuclideanDistance; // default euclidean distance
        std::function<float(float, float)> updateFn_;
    };

}

