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


#include <limits>
#include <thread>


namespace aerox{
    class PID {
    public:
        struct PIDParams{
            float kp, ki, kd, sat, wind;
        };

        PID(float _kp, float _ki, float _kd,
            float _minSat = std::numeric_limits<float>::min(),
            float _maxSat = std::numeric_limits<float>::max(),
            float _minWind = std::numeric_limits<float>::min(),
            float _maxWind = std::numeric_limits<float>::max());
        
        float update(float _val, float _incT);
    
        void enableRosPublisher(std::string _topic);
        void enableRosSubscriber(std::string _topic);
        void enableFastcomPublisher(int _port);
        void enableFastcomSubscriber(int _port);

        float reference() { return mReference; }
        void reference(float _ref) { mReference = _ref; mAccumErr = 0; mLastError = 0; mLastResult = 0; mBouncingFactor = 0.1;}
    
        float kp() const { return mKp; }
        float ki() const { return mKi; }
        float kd() const { return mKd; }
    
        void kp(float _kp) { mKp = _kp; }
        void ki(float _ki) { mKi = _ki; }
        void kd(float _kd) { mKd = _kd; }
    
        void setSaturations(float _min, float _max) { mMinSat = _min; mMaxSat = _max; }
        void getSaturations(float _min, float _max) { _min = mMinSat; _max = mMaxSat; }
    
        void setWindupTerms(float _min, float _max) { mWindupMin = _min; mWindupMax = _max; }
        void getWindupTerms(float _min, float _max) { _min = mWindupMin; _max = mWindupMax; }

    private:
        float mReference;
        float mKp, mKi, mKd;
        float mMinSat, mMaxSat;
        float mWindupMin, mWindupMax;
        float mLastResult, mLastError, mAccumErr;
        double mBouncingFactor = 0.1;
        bool run_ = false;
        std::thread mParamPubThread;
        int mPort;
        int ignoreCounter = 0;
    };

}


