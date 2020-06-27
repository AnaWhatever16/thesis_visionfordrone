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



#ifndef AEROXSUITE_UTILS_RATECOUNTER_H_
#define AEROXSUITE_UTILS_RATECOUNTER_H_

#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable> 

namespace aerox{

    /// Simple class for estimating Hz of loops
    /// @ingroup aerox_suite
    class RateCounter{
    public:
        RateCounter();
        ~RateCounter();

        void update();

        float hz();
        
    private:
        void counterLoop();

    private:
        std::thread counterThread_;
        std::mutex timeLord_;
        std::chrono::steady_clock::time_point t0_;
        std::condition_variable cv_;
        bool run_ = true;

        float hz_ = 0;
    };
}

#endif