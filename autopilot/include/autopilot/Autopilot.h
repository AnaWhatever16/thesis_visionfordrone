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


#ifndef AEROXSUITE_HAL_AUTOPILOT_H_
#define AEROXSUITE_HAL_AUTOPILOT_H_

#include <map>
#include <string>

#include <Eigen/Eigen>

namespace aerox{
    /// Available states for autopilots. 
    enum class AUTOPILOT_STATE {    UNKNOWN,
                                    IDLE, 
                                    MANUAL_FLIGHT, 
                                    AUTONOMOUS_FLIGHT };

    /// Abstract class of Autopilot. It provides for the strict required interface for aerox flight functionalities.
    /// @ingroup aerox_suite
    class Autopilot{
    public:
        /// Abstract method for initializing the autopilots.
        /// \param _params map of string containing specific configuration for the autopilot.
        /// \return true if intialized properly false if not.
        virtual bool init(std::map<std::string, std::string> _params) = 0;

        /// Method to check if the application is connected to the autopilot.
        /// \return true if connected false if not.
        virtual bool isConnected() = 0;

        /// Return current state of the autopilot.
        /// \return state of the autopilot in category of AUTOPILOT_STATE.
        AUTOPILOT_STATE state();

        /// Set a target position for the  UAV to hover on.
        /// \param _position: Target position for the UAV.
        /// \param _yaw: target orientation of the UAV.
        virtual void targetPosition(const Eigen::Vector3f &_position, const float _yaw) = 0;

        /// Set a target speed for the robot
        /// \param _speed: Target peed for the UAV
        /// \param _yaw: target orientation speed of the UAV.
        virtual void targetSpeed(const Eigen::Vector3f &_speed, const float _yawRate) = 0;

        /// Set a target rool, pitch and yaw for the autopilot. For the altitude, thrust is used.
        /// Please see specific implementation of Autopilot before using this method.
        /// \param _rpy: Target peed for the UAV.
        /// \param _thrust: target thrust for autopilot.
        virtual void targetRpyThrust(const Eigen::Vector3f &_rpy, const float _thrust) = 0;

        /// Set a target rool, pitch and yaw for the autopilot. This method intend to hold the target  directly.
        /// Please see specific implementation of Autopilot before using this method. This method is not granted to be
        /// implemented in all autopilots.
        /// \param _rpy: Target peed for the UAV.
        /// \param _alatitude: target thrust for autopilot.
        virtual void targetRpyAltitude(const Eigen::Vector3f &_rpy, const float _altitude) = 0;

        /// Send take off command to autopilot. It is not granted that all the autopilots go to the given altitude when
        /// taking off. 
        /// \param _altitude: target altitude for the takeoff. 
        virtual bool takeOff(const float _altitude) = 0;

        /// Send a land command to autopilot.
        virtual void land() =0;

        /// Get position estimate from autopilot.
        virtual Eigen::Vector3f position() = 0;

        /// Get position estimate from autopilot in GPS format latitude, longitude, altitude.
        virtual Eigen::Vector3f positionGps() { return {0,0,0}; };
        
        /// Get yaw estimate from autopilot.
        virtual float yaw() = 0;

        /// Get pose estimate from autopilot.
        virtual Eigen::Matrix4f pose() = 0;

        /// Get num of GPS
        virtual int numGps() {return 0;};

        /// Get if GPS signal is reliable.
        virtual bool gpsReliable() {return false; };

        /// Get info from RC channels
        virtual std::vector<float> rcChannels() { return {}; };

        virtual bool isFlying() { return false; };

    protected:
        AUTOPILOT_STATE currentState_ = AUTOPILOT_STATE::UNKNOWN;

    };
}

#endif