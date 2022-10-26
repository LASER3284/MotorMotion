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
 * @file TalonFXMotion.h
 * @brief 
 *      This file contains the declaration of the TalonFXMotion class, which 
 *      implements MotorMotion for TalonFX/Falcon 500
 */
#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include "laser/MotorMotion.h"
////////////////////////////////////////////////////////////////////////////////

namespace laser {

/**
 * @namespace talonfx
 * @brief 
 *      The namespace containing TalonFXMotion implementation
 */
namespace talonfx {

    /**
     * @namespace defaults
     * @brief 
     *      This namespace is meant to contain defaults and constants for the 
     *      TalonFXMotion class implementation
     */
    namespace defaults {
        /**
         * @brief 
         *      The number of sensor units per revolution of the input shaft
         */
        constexpr double countsPerRev = 2048.0;
    } // namespace defaults

    /**
     * @class TalonFXMotion
     * @brief 
     *      This is the declaration of the TalonFXMotion class, which
     *      implements MotorMotion
     */
    class TalonFXMotion : public MotorMotion<ctre::phoenix::ErrorCode, ctre::phoenix::motorcontrol::can::WPI_TalonFX> {
        public:
            /**
             * @brief 
             *      Constructor that accepts the device ID on the CAN bus
             * @param deviceID
             *      The ID on the CAN bus to use for the TalonFX
             * @param ratio
             *      The gear ratio to be used internally for converting from m 
             *      and m/s to encoder ticks (input:output)
             * @param diameter
             *      The wheel diameter in meters to be used internally for math
             *      regarding motor speed and position
             */
            TalonFXMotion(int /* deviceID */, double /* gearing */, units::meter_t /* diameter */);

            /**
             * @brief 
             *      Destructor for the class; deletes any stray pointers
             */
            ~TalonFXMotion();

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
            void ConfigLimitSwitches(bool /* isFwdNO */, bool /* isRevNO */) override;

            /**
             * @brief 
             *      Configure the current limit (in Amps) of the motor
             * @param amps
             *      The amperage limit of the motor in units::ampere_t
             * @return 
             *      The error reported by the motor controller, based on the 
             *      derived class's inheritance template
             */
            ctre::phoenix::ErrorCode ConfigCurrentLimit(units::ampere_t /* amps */) override;

            /**
             * @brief 
             *      Halts the motor as quickly as the open-loop ramp rate allows
             */
            void Stop() override;

            /**
             * @brief 
             *      Clears sticky faults on the motor controller (not typically
             *      used for REV devices)
             */
            void ClearStickyFaults() override;

            /**
             * @brief 
             *      Returns the position the motor has traveled based on 
             *      encoder counts
             * @return 
             *      units::meter_t representing the distance traveled in meters
             */
            units::meter_t GetActualPosition() override;

            /**
             * @brief 
             *      Returns the velocity the wheel is currently spinning at in
             *      meters per second based on the encoder velocity
             * @return 
             *      units::meters_per_second_t representing the velocity in m/s
             */
            units::meters_per_second_t GetActualVelocity() override;

            /**
             * @brief 
             *      Returns the angular velocity the motor shaft is currently 
             *      spinning at in radians per second based on the encoder 
             *      velocity
             * @return 
             *      units::radians_per_second_t representing the angular 
             *      velocity in rad/s
             */
            units::radians_per_second_t GetActualAngularVelocity() override;

            /**
             * @brief 
             *      Returns the tolerance of the position in meters
             * @return 
             *      The max tolerance of the position; how far off the actual
             *      is liable to be from the setpoint
             */
            units::meter_t GetPositionTolerance() override;

            /**
             * @brief 
             *      Returns the tolerance of the velocity in m/s
             * @return 
             *      The max tolerance of the velocity; how far off the actual 
             *      is liable to be from the setpoint
             */
            units::meters_per_second_t GetVelocityTolerance() override;

            /**
             * @brief 
             *      Returns the tolerance of the velocity in rad/s
             * @return 
             *      The max tolerance of the velocity; how far off the actual 
             *      is liable to be from the setpoint
             */
            units::radians_per_second_t GetAngularVelocityTolerance() override;

            /**
             * @brief 
             *      Sets whether the motor is to spin opposite of the default 
             *      direction
             * @param isInverted
             *      When true, the motor is to be inverted
             */
            void SetMotorInverted(bool /* isInverted */) override;

            /**
             * @brief 
             *      Sets the maximum ramp rate when in a closed feedback loop
             * @param rate
             *      The ramp rate, in seconds, for the closed feedback loop
             */
            void SetClosedLoopRampRate(double /* rate */) override;

            /**
             * @brief 
             *      Sets the maximum ramp rate when in an open feedback loop
             * @param rate
             *      The ramp rate, in seconds, for the open feedback loop
             */
            void SetOpenLoopRampRate(double /* rate */) override;

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
            void SetPIDValues(
                double /* proportional */, 
                double /* integral */, 
                double /* derivative */, 
                double /* feedforward */
            ) override;

            /**
             * @brief 
             *      Sets the maximum tolerance for the position setpoint
             * @param tolerance
             *      The maximum tolerance in units::meter_t
             */
            void SetTolerance(units::meter_t /* tolerance */) override;

            /**
             * @brief 
             *      Sets the maximum tolerance for the linear velocity setpoint
             * @param tolerance
             *      The maximum tolerance in units::meters_per_second_t
             */
            void SetTolerance(units::meters_per_second_t /* tolerance */) override;

            /**
             * @brief 
             *      Sets the maximum tolerance for the angular velocity setpoint
             * @param tolerance
             *      The maximum tolerance in units::radians_per_second_t
             */
            void SetTolerance(units::radians_per_second_t /* tolerance */) override;

            /**
             * @brief 
             *      Returns the motor voltage
             * @return 
             *      units::volt_t representing the motor voltage in Volts
             */
            units::volt_t GetMotorVoltage() override;

            /**
             * @brief 
             *      Sets the motor voltage
             * @param voltage
             *      units::volt_t representing the motor voltage in Volts
             */
            void SetMotorVoltage(units::volt_t /* voltage */) override;

            /**
             * @brief 
             *      Returns the motor amperage
             * @return
             *      units::ampere_t representing the motor amperage in Amperes
             */
            units::ampere_t GetMotorCurrent() override;

            /**
             * @brief 
             *      Returns the raw number of encoder counts that have been 
             *      traveled
             * @return 
             *      Integer value representing the number of encoder counts
             */
            int GetRawEncoderCounts() override;

            /**
             * @brief 
             *      Sets the max speed the closed loop controller is allowed to
             *      reach the setpoint; this will limit how quickly the motor 
             *      will ramp up/down to the setpoint
             * @param time
             *      Time in units::second_t of the fastest time the closed loop
             *      controller is allowed
             */
            void SetClosedRampRate(units::second_t /* time */) override;

            /**
             * @brief 
             *      Sets the max speed the open loop controller is allowed to 
             *      set the motor voltage; this will limit how quickly the 
             *      voltage will ramp
             * @param time
             *      Time in units::second_t of the fastest time the open loop 
             *      controller is allowed
             */
            void SetOpenRampRate(units::second_t /* time */) override;

            /**
             * @brief 
             *      Sets the setpoint for the position of the motor in meters
             * @param position
             *      Desired position of the motor in units::meter_t
             */
            void SetSetpoint(units::meter_t /* position */) override;

            /**
             * @brief 
             *      Sets the setpoint for the linear velocity of the motor in 
             *      m/s
             * @param lvelocity
             *      Desired linear velocity of the motor in 
             *      units::meters_per_second_t
             */
            void SetSetpoint(units::meters_per_second_t /* lvelocity */) override;

            /**
             * @brief 
             *      Sets the setpoint for the angular velocity of the motor in
             *      rad/s
             * @param avelocity
             *      Desired angular velocity of the motor in 
             *      units::radians_per_second_t
             */
            void SetSetpoint(units::radians_per_second_t /* avelocity */) override;

            /**
             * @brief 
             *      Sets the Integral Zone for error in units per millisecond;
             *      because it doesn't use the Units library for unit checking,
             *      make sure your units are correct
             * @param izone
             *      Desired IZone left as a primitive double
             */
            void SetAccumIZone(double /* izone */) override;

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
            void SetPositionSoftLimits(units::meter_t /* minpos */, units::meter_t /* maxpos */) override;

            /**
             * @brief 
             *      Returns the state of the reverse limit switch
             * @return 
             *      Boolean value, true = pressed, false = unpressed
             */
            bool IsRevLimitSwitchPressed() override;

            /**
             * @brief 
             *      Returns the state of the forward limit switch
             * @return 
             *      Boolean value, true = pressed, false = unpressed
             */
            bool IsFwdLimitSwitchPressed() override;

            /**
             * @brief 
             *      Stops the motor and resets the encoder to 0
             */
            void Reset() override;
    }; // class TalonFXMotion

} // namespace talonfx

} // namespace laser
