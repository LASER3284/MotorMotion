/*
Copyright 2022 Camdenton LASER 3284

This file is part of MotorMotion.

MotorMotion is free software: you can redistribute it and/or modify it under 
the terms of the GNU Lesser General Public License as published by the Free 
Software Foundation, either version 3 of the License, or (at your option) any 
later version.

MotorMotion is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along 
with MotorMotion. If not, see <https://www.gnu.org/licenses/>. 
*/

/**
 * @file MotorMotion.h
 * @brief 
 *      This file contains the mostly virtual class MotorMotion, which is 
 *      implemented by other classes.
 */
#pragma once

#include <units/angle.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/length.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

namespace laser {
    /**
     * @enum SetpointType
     * @brief 
     *      Which unit and set of PID values to use for reaching the setpoint 
     *      within the MotorMotion class
     */
    enum SetpointType {
        /** @brief No setpoint in use */
        eNone,
        /** @brief Desired position in meters */
        ePosition,
        /** @brief Desired velocity in meters per second */
        eLinearVelocity,
        /** @brief Desired angular velocity in radians per second */
        eAngularVelocity
    }; // enum SetpointType

    /**
     * @class MotorMotion
     * @brief 
     *      A basic, mostly virtual, class that follows a template for the 
     *      ErrorEnum of the MotorType class
    */
    template <typename ErrorEnum, class MotorType>
    class MotorMotion {
        public:
            /* Virtual methods that tend to depend on MotorType */

            /**
             * @brief 
             *      Configure whether limit switches are NO or NC
             * @param isFwdNO
             *      When true, the Forward Limit Switch will be treated as 
             *      Normally Open
             * @param isRevNO
             *      When true, the Reverse Limit Switch will be treated as 
             *      Normally Open
             */
            virtual void ConfigLimitSwitches(bool /* isFwdNO */, bool /* isRevNO */);

            /**
             * @brief 
             *      Configure the current limit (in Amps) of the motor
             * @param amps
             *      The amperage limit of the motor in units::ampere_t
             * @return 
             *      The error reported by the motor controller, based on the 
             *      derived class's inheritance template
             */
            virtual ErrorEnum ConfigCurrentLimit(units::ampere_t /* amps */);

            /**
             * @brief 
             *      Halts the motor as quickly as the open-loop ramp rate allows
             */
            virtual void Stop();

            /**
             * @brief 
             *      Clears sticky faults on the motor controller (not typically
             *      used for REV devices)
             */
            virtual void ClearStickyFaults();

            /**
             * @brief 
             *      Returns the position the motor has traveled based on 
             *      encoder counts
             * @return 
             *      units::meter_t representing the distance traveled in meters
             */
            virtual units::meter_t GetActualPosition();

            /**
             * @brief 
             *      Returns the velocity the wheel is currently spinning at in
             *      meters per second based on the encoder velocity
             * @return 
             *      units::meters_per_second_t representing the velocity in m/s
             */
            virtual units::meters_per_second_t GetActualVelocity();

            /**
             * @brief 
             *      Returns the angular velocity the motor shaft is currently 
             *      spinning at in radians per second based on the encoder 
             *      velocity
             * @return 
             *      units::radians_per_second_t representing the angular 
             *      velocity in rad/s
             */
            virtual units::radians_per_second_t GetActualAngularVelocity();

            /**
             * @brief 
             *      Returns the tolerance of the position in meters
             * @return 
             *      The max tolerance of the position; how far off the actual
             *      is liable to be from the setpoint
             */
            virtual units::meter_t GetPositionTolerance();

            /**
             * @brief 
             *      Returns the tolerance of the velocity in m/s
             * @return 
             *      The max tolerance of the velocity; how far off the actual 
             *      is liable to be from the setpoint
             */
            virtual units::meters_per_second_t GetVelocityTolerance();

            /**
             * @brief 
             *      Returns the tolerance of the velocity in rad/s
             * @return 
             *      The max tolerance of the velocity; how far off the actual 
             *      is liable to be from the setpoint
             */
            virtual units::radians_per_second_t GetAngularVelocityTolerance();

            /**
             * @brief 
             *      Sets whether the motor is to spin opposite of the default 
             *      direction
             * @param isInverted
             *      When true, the motor is to be inverted
             */
            virtual void SetMotorInverted(bool /* isInverted */);

            /**
             * @brief 
             *      Sets the maximum ramp rate when in a closed feedback loop
             * @param rate
             *      The ramp rate, in seconds, for the closed feedback loop
             */
            virtual void SetClosedLoopRampRate(double /* rate */);

            /**
             * @brief 
             *      Sets the maximum ramp rate when in an open feedback loop
             * @param rate
             *      The ramp rate, in seconds, for the open feedback loop
             */
            virtual void SetOpenLoopRampRate(double /* rate */);

            /**
             * @brief 
             *      Sets the PIDF values for the controller; each setpoint type
             *      has its own PID values, this will feed it to the currently 
             *      active one (default/none: they all get the same PID values)
             * @param proportional
             *      The desired proportional gain
             * @param integral
             *      The desired integral gain
             * @param derivative
             *      The desired derivative gain
             * @param feedforward
             *      The desired feed forward gain
             */
            virtual void SetPIDValues(
                double /* proportional */, 
                double /* integral */, 
                double /* derivative */, 
                double /* feedforward */
            );

            /**
             * @brief 
             *      Sets the maximum tolerance for the position setpoint
             * @param tolerance
             *      The maximum tolerance in units::meter_t
             */
            virtual void SetTolerance(units::meter_t /* tolerance */);

            /**
             * @brief 
             *      Sets the maximum tolerance for the linear velocity setpoint
             * @param tolerance
             *      The maximum tolerance in units::meters_per_second_t
             */
            virtual void SetTolerance(units::meters_per_second_t /* tolerance */);

            /**
             * @brief 
             *      Sets the maximum tolerance for the angular velocity setpoint
             * @param tolerance
             *      The maximum tolerance in units::radians_per_second_t
             */
            virtual void SetTolerance(units::radians_per_second_t /* tolerance */);

            /**
             * @brief 
             *      Returns the motor voltage
             * @return 
             *      units::volt_t representing the motor voltage in Volts
             */
            virtual units::volt_t GetMotorVoltage();

            /**
             * @brief 
             *      Sets the motor voltage
             * @param voltage
             *      units::volt_t representing the motor voltage in Volts
             */
            virtual void SetMotorVoltage(units::volt_t /* voltage */);

            /**
             * @brief 
             *      Returns the motor amperage
             * @return
             *      units::ampere_t representing the motor amperage in Amperes
             */
            virtual units::ampere_t GetMotorCurrent();

            /**
             * @brief 
             *      Returns the raw number of encoder counts that have been 
             *      traveled
             * @return 
             *      Integer value representing the number of encoder counts
             */
            virtual int GetRawEncoderCounts();

            /**
             * @brief 
             *      Sets the max speed the closed loop controller is allowed to
             *      reach the setpoint; this will limit how quickly the motor 
             *      will ramp up/down to the setpoint
             * @param time
             *      Time in units::second_t of the fastest time the closed loop
             *      controller is allowed
             */
            virtual void SetClosedRampRate(units::second_t /* time */);

            /**
             * @brief 
             *      Sets the max speed the open loop controller is allowed to 
             *      set the motor voltage; this will limit how quickly the 
             *      voltage will ramp
             * @param time
             *      Time in units::second_t of the fastest time the open loop 
             *      controller is allowed
             */
            virtual void SetOpenRampRate(units::second_t /* time */);

            /**
             * @brief 
             *      Sets the setpoint for the position of the motor in meters
             * @param position
             *      Desired position of the motor in units::meter_t
             */
            virtual void SetSetpoint(units::meter_t /* position */);

            /**
             * @brief 
             *      Sets the setpoint for the linear velocity of the motor in 
             *      m/s
             * @param lvelocity
             *      Desired linear velocity of the motor in 
             *      units::meters_per_second_t
             */
            virtual void SetSetpoint(units::meters_per_second_t /* lvelocity */);

            /**
             * @brief 
             *      Sets the setpoint for the angular velocity of the motor in
             *      rad/s
             * @param avelocity
             *      Desired angular velocity of the motor in 
             *      units::radians_per_second_t
             */
            virtual void SetSetpoint(units::radians_per_second_t /* avelocity */);

            /**
             * @brief 
             *      Sets the Integral Zone for error in units per millisecond;
             *      this is in revolutions of the output shaft only
             * @param _ISzone
             *      Desired IZone left as a primitive double
             */
            virtual void SetAccumIZone(double /* _izone */);

            /**
             * @brief 
             *      Sets the positional soft limits in meters; the motor will 
             *      not intentionally leave the interval of values, [minpos, 
             *      maxpos] 
             * @param minpos
             *      Minimum position value in units::meter_t, must be less
             *      than maxpos
             * @param maxpos
             *      Maximum position values in units::meter_t, must be greater
             *      than minpos
             */
            virtual void SetPositionSoftLimits(units::meter_t /* minpos */, units::meter_t /* maxpos */);

            /**
             * @brief 
             *      Returns the state of the reverse limit switch
             * @return 
             *      Boolean value, true = pressed, false = unpressed
             */
            virtual bool IsRevLimitSwitchPressed();

            /**
             * @brief 
             *      Returns the state of the forward limit switch
             * @return 
             *      Boolean value, true = pressed, false = unpressed
             */
            virtual bool IsFwdLimitSwitchPressed();

            /**
             * @brief 
             *      Stops the motor and resets the encoder to 0
             */
            virtual void Reset();

            /* One liners - non-virtual */

            /**
             * @brief 
             *      Returns the pointer to the MotorType class for specific use
             *      cases
             * @return 
             *      Pointer to class MotorType
             */
            MotorType* GetMotorPointer() { return motor; }

            /**
             * @brief 
             *      Returns the state of the reverse limit switch
             * @return 
             *      Boolean value, true = pressed, false = unpressed
             */
            bool IsReady() { return isReady; }

            /**
             * @brief 
             *      Sets the motor output percentage
             * @param percent
             *      A percentage from [-1, 1]; exceeding this interval may
             *      cause undefined behavior
             */
            void Set(double percent) { motor->Set(percent); }

            /**
             * @brief 
             *      Returns the setpoint type currently in use; this is 
             *      determined by previously called methods
             * @return 
             *      The setpoint type currently in use; see the SetpointType 
             *      enum for what these types are
             */
            SetpointType GetSetpointType() { return setpointType; }

            /**
             * @brief 
             *      Returns the current position setpoint; use 
             *      GetSetpointType() to check if this is the active setpoint
             * @return 
             *      The distance of the position setpoint in units::meter_t
             */
            units::meter_t GetPositionSetpoint() { return positionSetpoint; }

            /**
             * @brief 
             *      Returns th current linear velocity setpoint; use 
             *      GetSetpointType() to check if this is the active setpoint
             * @return 
             *      The velocity of the linear velocity setpoint in 
             *      units::meters_per_secont_t
             */
            units::meters_per_second_t GetVelocitySetpoint() { return velocitySetpoint; }

            /**
             * @brief 
             *      Returns the angular velocity setpoint; use 
             *      GetSetpointType() to check if this is the active setpoint
             * @return 
             *      The angular velocity of the angular velocity setpoint in 
             *      units::radians_per_second_t
             */
            units::radians_per_second_t GetAngularVelocitySetpoint() { return avelSetpoint; }

            /**
             * @brief 
             *      Returns the last recorded error, if any
             * @return 
             *      Type based on template of the class; the last error known 
             *      to have occurred
             */
            ErrorEnum GetLastError() { return lastError; }

            /**
             * @brief 
             *      Sets the gear ratio to be used for calculations between 
             *      unit conversions of the wheel and actual motor rotation
             * @param ratio
             *      The desired gear ratio as a decimal value (output to input)
             */
            void SetGearing(double ratio) { gearing = ratio; }

            /**
             * @brief 
             *      Returns the configured gear ratio as a decimal value
             * @return 
             *      The gear ratio currently being used to determine how motor 
             *      rotation correlates to distance and velocity
             */
            double GetGearing() { return gearing; }

            /**
             * @brief 
             *      Sets the desired wheel diameter, in meters, to be used for 
             *      calculating distances and velocities
             * @param diameter
             *      Desired wheel diameter in units::meter_t
             */
            void SetWheelDiameter(units::meter_t diameter) { wheelDiameter = diameter; }

            units::meter_t GetWheelDiameter() { return wheelDiameter; }

        protected:
            /**
             * @brief 
             *      Pointer to class MotorType, based on template of the class
             */
            MotorType* motor;

            /**
             * @brief 
             *      Last error to have known to occurred
             */
            ErrorEnum lastError;

            /**
             * @brief 
             *      SetpointType enum type recording the currently in-use 
             *      setpoint
             */
            SetpointType setpointType;

            /* Primitive member types */

            /**
             * @brief 
             *      Is forward limit switch Normally Open?
             */
            bool isFwdLimitSwitchNO;

            /**
             * @brief 
             *      Is reverse limit switch Normally Open?
             */
            bool isRevLimitSwitchNO;

            /**
             * @brief 
             *      Current position setpoint in units::meter_t; this describes
             *      how far the wheel should travel compared to its position 
             *      when first initializing
             */
            units::meter_t positionSetpoint;

            /**
             * @brief 
             *      Current linear velocity setpoint in 
             *      units::meters_per_second_t; this describes how fast the 
             *      wheel should move
             */
            units::meters_per_second_t velocitySetpoint;

            /**
             * @brief 
             *      Current angular velocity setpoint in 
             *      units::radians_per_second_t; this describes how fast the 
             *      output shaft should spin (2pi rad/s = 1/60 rpm)
             */
            units::radians_per_second_t avelSetpoint;

            /**
             * @brief 
             *      Proportional gain for the closed feedback controller for 
             *      position
             */
            double positionProportional;

            /**
             * @brief 
             *      Integral gain for the closed feedback controller for
             *      position
             */
            double positionIntegral;
            
            /**
             * @brief 
             *      Derivative gain for the closed feedback controller for 
             *      position
             */
            double positionDerivative;

            /**
             * @brief 
             *      Feed forward gain for the closed feedback controller for 
             *      position
             */
            double positionFeedForward;

            /**
             * @brief 
             *      Maximum allowed error of the setpoint; this should be as 
             *      small as your PID values are accurate
             */
            units::meter_t positionTolerance;

            /**
             * @brief 
             *      Proportional gain for the closed feedback controller for 
             *      linear velocity
             */
            double velocityProportional;

            /**
             * @brief 
             *      Integral gain for the closed feedback controller for 
             *      linear velocity
             */
            double velocityIntegral;

            /**
             * @brief 
             *      Derivative gain for the closed feedback controller for 
             *      linear velocity
             */
            double velocityDerivative;

            /**
             * @brief 
             *      Feed forward gain for the closed feedback controller for 
             *      linear velocity
             */
            double velocityFeedForward;

            /**
             * @brief 
             *      Maximum allowed error of the setpoint; this should be as 
             *      small as your PID values are accurate
             */
            units::meters_per_second_t velocityTolerance;

            /**
             * @brief 
             *      Proportional gan of the closed feedback controller for 
             *      angular velocity
             */
            double avelProportional;

            /**
             * @brief 
             *      Integral gain for the closed feedback controller for 
             *      angular velocity
             */
            double avelIntegral;

            /**
             * @brief 
             *      Derivative gain for the closed feedback controller for 
             *      angular velocity
             */
            double avelDerivative;

            /**
             * @brief 
             *      Feed forward gain for the closed feedback controller for 
             *      angular velocity
             */
            double avelFeedForward;

            /**
             * @brief 
             *      Maximum allowed error of the setpoint; this should be as 
             *      small as your PID values are accurate
             */
            units::radians_per_second_t avelTolerance;

            /**
             * @brief 
             *      Farthest back the motor is allowed to travel; when set to 
             *      zero, this is ignored
             */
            units::meter_t lowerPositionSoftLimit;

            /**
             * @brief 
             *      Farthest forward the motor is allowed to travel; when set 
             *      to zero, this is ignored
             */
            units::meter_t upperPositionSoftLimit;

            /**
             * @brief 
             *      Slowest the wheel is allowed to move (when negative this
             *      is max reverse speed); when set to zero, this is ignored
             */
            units::meters_per_second_t lowerVelocitySoftLimit;

            /**
             * @brief 
             *      Fastest the wheel is allowed to move (must be greater than 
             *      lowerVelocitySoftLimit); when set to zero, this is ignored
             */
            units::meters_per_second_t upperVelocitySoftLimit;

            /**
             * @brief 
             *      Global integral zone of error in units per millisecond; 
             *      units are not checked!
             */
            double izone;

            /**
             * @brief 
             *      Gear ratio used to convert from m and m/s to encoder ticks
             */
            double gearing;

            /**
             * @brief 
             *      Wheel diameter (in meters) used for calculations on distance and 
             *      velocity
             */
            units::meter_t wheelDiameter;

            /**
             * @brief 
             *      Device ID on the CAN bus; passed into the constructor
             */
            int deviceID;
    }; // class MotorMotion

} // namespace laser
