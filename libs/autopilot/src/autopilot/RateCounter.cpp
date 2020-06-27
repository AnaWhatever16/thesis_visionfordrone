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

#include <autopilot/RateCounter.h>
#include <iostream>

namespace aerox{

    RateCounter::RateCounter(){
        counterThread_ = std::thread(&RateCounter::counterLoop, this);
    }
    RateCounter::~RateCounter(){
        run_ = false;
        cv_.notify_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(counterThread_.joinable())
            counterThread_.join();
    }


    void RateCounter::update(){
        cv_.notify_all();
    }

    float RateCounter::hz(){
        return hz_;
    }
    
    void RateCounter::counterLoop(){
        std::unique_lock<std::mutex> lck(timeLord_);
        while (run_){
            cv_.wait(lck);
            auto t1 = std::chrono::steady_clock::now();
            float time = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0_).count();
            t0_ = t1;
            hz_ = 1e6/time;
        } 
    }

}
