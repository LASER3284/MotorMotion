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
 *      implements MotorMotion for TalonFX/Falcon 500 motors.
 * 
 * This allows for a common mode of control between different motor controllers 
 * using a similar API. This can often be useful for controlling various motor 
 * types on a robot.
 * @see MotorMotion.h
 */
#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include "laser/MotorMotion.h"
////////////////////////////////////////////////////////////////////////////////

namespace laser {

/**
 * @brief 
 *      The namespace containing TalonFXMotion implementation.
 * 
 * This implementation involves using the CTRE Phoenix API to control 
 * TalonFX/Falcon 500 motors. This makes them similar to control to Spark 
 * Max/NEO motors due to the common inheritance of the MotorMotion class, which 
 * can often be useful.
 * @see MotorMotion.h
 */
namespace talonfx {

    /**
     * @brief 
     *      This namespace is meant to contain defaults and constants for the 
     *      TalonFXMotion class implementation.
     * 
     * There's currently only one member of the namespace; however, future 
     * releases may add more.
     */
    namespace defaults {
        /**
         * @brief 
         *      The number of sensor units per revolution of the input shaft.
         * 
         * By default, the Falcon 500, which exclusively uses the TalonFX 
         * controller, has an integrated sensor with a CPR of 2048, which is 
         * used by default in the TalonFXMotion implementation.
         */
        constexpr double countsPerRev = 2048.0;
    } // namespace defaults

    /**
     * @class TalonFXMotion TalonFXMotion.h laser/TalonFXMotion.h
     * @brief 
     *      This is the declaration of the TalonFXMotion class, which
     *      implements MotorMotion.
     * 
     * This implementation involves using the CTRE Phoenix API to control 
     * TalonFX/Falcon 500 motors. This makes them similar to control to Spark 
     * Max/NEO motors due to the common inheritance of the MotorMotion class, which 
     * can often be useful.
     * @see MotorMotion
     */
    class TalonFXMotion : public MotorMotion<ctre::phoenix::ErrorCode, ctre::phoenix::motorcontrol::can::WPI_TalonFX> {
        public:
            /**
             * @brief 
             *      Constructor that accepts the device ID on the CAN bus.
             * 
             * Of the parameters this constructor accepts, only the device ID is
             * required, as this specifies with CAN device is the TalonFX to be 
             * used as the motor instance for the object. The defaults for the 
             * other two parameters are both 1.0 (1.0_m for the diameter of the 
             * wheel). This allows 1:1 systems that just care about rotational 
             * speed to ignore the wheel diameter.
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
             *      Destructor for the class; deletes any stray pointers.
             * 
             * Generally this doesn't need to be directly called, it will be 
             * called in the destructor of the main class through the delete 
             * call.
             */
            ~TalonFXMotion();

            /**
             * @brief 
             *      Configure whether limit switches are NO or NC.
             * 
             * By default, both limit switches are Normally Open (NO), but can 
             * be configured to by Normally Closed (NC) through this method. 
             * This is only useful if you need to check the limit switches.
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
             *      Configure the current limit (in Amps) of the motor.
             * 
             * This method is generally more useful for SparkMaxMotion, since 
             * Falcon 500 motors can handle more current than any other 
             * FRC-legal motor type.
             * @param amps
             *      The amperage limit of the motor in units::ampere_t
             * @return 
             *      The error reported by the motor controller, based on the 
             *      derived class's inheritance template
             */
            ctre::phoenix::ErrorCode ConfigCurrentLimit(units::ampere_t /* amps */) override;

            /**
             * @brief 
             *      Halts the motor as quickly as the open-loop ramp rate allows.
             * 
             * This is useful for a psuedo E-stop that's more recoverable and/or
             * only needs to apply to a specific part, such as the motor 
             * controlled by the MotorMotion instance.
             */
            void Stop() override;

            /**
             * @brief 
             *      Returns the position the motor has traveled based on 
             *      encoder counts.
             * 
             * This is typically used when cross referencing a setpoint.
             * @return 
             *      units::meter_t representing the distance traveled in meters
             */
            units::meter_t GetActualPosition() override;

            /**
             * @brief 
             *      Returns the velocity the wheel is currently spinning at in
             *      meters per second based on the encoder velocity
             * 
             * This is typically used when cross referencing a setpoint.
             * @return 
             *      units::meters_per_second_t representing the velocity in m/s
             */
            units::meters_per_second_t GetActualVelocity() override;

            /**
             * @brief 
             *      Returns the angular velocity the motor shaft is currently 
             *      spinning at in radians per second based on the encoder 
             *      velocity.
             * 
             * This is typically used when cross referencing a setpoint.
             * @return 
             *      units::radians_per_second_t representing the angular 
             *      velocity in rad/s
             */
            units::radians_per_second_t GetActualAngularVelocity() override;

            /**
             * @brief 
             *      Returns the tolerance of the position in meters.
             * 
             * This is how far off the controller is allowed to be from the 
             * setpoint. This is configured into the closed loop controller of 
             * the motor controller.
             * @return 
             *      The max tolerance of the position; how far off the actual
             *      is liable to be from the setpoint
             */
            units::meter_t GetPositionTolerance() override;

            /**
             * @brief 
             *      Returns the tolerance of the velocity in m/s.
             * 
             * This is how far off the controller is allowed to be from the 
             * setpoint.
             * @return 
             *      The max tolerance of the velocity; how far off the actual 
             *      is liable to be from the setpoint
             */
            units::meters_per_second_t GetVelocityTolerance() override;

            /**
             * @brief 
             *      Returns the tolerance of the velocity in rad/s.
             * 
             * This is how far off the controller is allowed to be from the 
             * setpoint.
             * @return 
             *      The max tolerance of the velocity; how far off the actual 
             *      is liable to be from the setpoint
             */
            units::radians_per_second_t GetAngularVelocityTolerance() override;

            /**
             * @brief 
             *      Sets whether the motor is to spin opposite of the default
             *      direction.
             * 
             * This method has not been tested to work with setpoints; whether 
             * the controller negates the setpoint to match direction is 
             * currently undefined.
             * @param isInverted
             *      When true, the motor is to be inverted
             */
            void SetMotorInverted(bool /* isInverted */) override;

            /**
             * @brief 
             *      Sets the PIDF values for the controller.
             * 
             * Each setpoint type has its own PID values, this method will feed 
             * it to the currently active one (default/none: no PIDF values are 
             * saved). PIDF is used for closed loop control of reaching a 
             * setpoint.
             * @param proportional
             *      The desired proportional gain
             * @param integral
             *      The desired integral gain
             * @param derivative
             *      The desired derivative gain
             * @param feedforward
             *      The desired feed forward gain
             * @see https://en.wikipedia.org/wiki/PID_controller
             */
            void SetPIDValues(
                double /* proportional */, 
                double /* integral */, 
                double /* derivative */, 
                double /* feedforward */
            ) override;

            /**
             * @brief 
             *      Sets the maximum tolerance for the position setpoint.
             * 
             * This is how far off the controller is allowed to be from the 
             * setpoint. This is configured into the closed loop controller of 
             * the motor controller.
             * @param tolerance
             *      The maximum tolerance in units::meter_t
             */
            void SetTolerance(units::meter_t /* tolerance */) override;

            /**
             * @brief 
             *      Sets the maximum tolerance for the linear velocity setpoint.
             * 
             * This is how far off the controller is allowed to be from the 
             * setpoint. This is configured into the closed loop controller of 
             * the motor controller.
             * @param tolerance
             *      The maximum tolerance in units::meters_per_second_t
             */
            void SetTolerance(units::meters_per_second_t /* tolerance */) override;

            /**
             * @brief 
             *      Sets the maximum tolerance for the angular velocity 
             *      setpoint.
             * 
             * This is how far off the controller is allowed to be from the 
             * setpoint. This is configured into the closed loop controller of 
             * the motor controller.
             * @param tolerance
             *      The maximum tolerance in units::radians_per_second_t
             */
            void SetTolerance(units::radians_per_second_t /* tolerance */) override;

            /**
             * @brief 
             *      Returns the motor voltage.
             * 
             * This method can be used for diagnostic purposes and determining 
             * if a motor controller is feeding the expected voltage.
             * @return 
             *      units::volt_t representing the motor voltage in Volts
             */
            units::volt_t GetMotorVoltage() override;

            /**
             * @brief 
             *      Sets the motor voltage.
             * 
             * This method is typically not used outside of diagnostics and 
             * testing.
             * @param voltage
             *      units::volt_t representing the motor voltage in Volts
             */
            void SetMotorVoltage(units::volt_t /* voltage */) override;

            /**
             * @brief 
             *      Returns the motor amperage.
             * 
             * This method is typically not used outside of diagnostics and 
             * testing
             * @return
             *      units::ampere_t representing the motor amperage in Amperes
             */
            units::ampere_t GetMotorCurrent() override;

            /**
             * @brief 
             *      Returns the raw number of encoder counts that have been 
             *      traveled.
             * 
             * The value returned can be used for checking math, but is 
             * otherwise generally not needed; there are existing methods that 
             * do such math.
             * @return 
             *      Integer value representing the number of encoder counts
             * @see GetActualPosition()
             * @see GetActualVelocity()
             * @see GetActualAngularVelocity()
             */
            int GetRawEncoderCounts() override;

            /**
             * @brief 
             *      Sets the max speed the closed loop controller is allowed to
             *      reach the setpoint; this will limit how quickly the motor 
             *      will ramp up/down to the setpoint.
             * 
             * The maximum rate at which the closed loop controller will reach 
             * the setpoint will be partially determined through 
             * PIDF/characterization tuning as well, as certain values may take 
             * longer to reach the setpoint.
             * @param time
             *      Time in units::second_t of the fastest time the closed loop
             *      controller is allowed
             * @see SetPIDValues()
             */
            void SetClosedRampRate(units::second_t /* time */) override;

            /**
             * @brief 
             *      Sets the max speed the open loop controller is allowed to 
             *      set the motor voltage; this will limit how quickly the 
             *      voltage will ramp.
             * 
             * This does not affect the PIDF controller, but is can be used when
             * something needs to stop relatively quickly or slowly.
             * @param time
             *      Time in units::second_t of the fastest time the open loop 
             *      controller is allowed
             */
            void SetOpenRampRate(units::second_t /* time */) override;

            /**
             * @brief 
             *      Sets the setpoint for the position of the motor in meters.
             * 
             * This is used to drive the motor to a specific setpoint; note that
             * the motor will attempt to reach the setpoint as fast as possible.
             * @param position
             *      Desired position of the motor in units::meter_t
             * @todo 
             *      Add velocity limits to the motor.
             */
            void SetSetpoint(units::meter_t /* position */) override;

            /**
             * @brief 
             *      Sets the setpoint for the linear velocity of the motor in 
             *      m/s.
             * 
             * This method is used to drive the motor at a constant velocity for
             * as long as the robot is enabled and the setpoint is constant.
             * @param lvelocity
             *      Desired linear velocity of the motor in 
             *      units::meters_per_second_t
             */
            void SetSetpoint(units::meters_per_second_t /* lvelocity */) override;

            /**
             * @brief 
             *      Sets the setpoint for the angular velocity of the motor in
             *      rad/s.
             * 
             * This method should not be confused with linear velocity; this 
             * method deals with rotation rather than distance. It will drive 
             * the motor at a constant rotational speed for as long as the robot
             * is enabled and the setpoint is constant.
             * @param avelocity
             *      Desired angular velocity of the motor in 
             *      units::radians_per_second_t
             */
            void SetSetpoint(units::radians_per_second_t /* avelocity */) override;

            /**
             * @brief 
             *      Sets the Integral Zone for error in units per millisecond.
             * 
             * This method will determine the factor by which the Integral part 
             * of the PID controller is effected by error over in an instant.
             * Because it doesn't use the Units library for unit checking, make 
             * sure your units are correct.
             * @param izone
             *      Desired IZone left as a primitive double
             * @todo 
             *      More documentation on usage.
             */
            void SetAccumIZone(double /* izone */) override;

            /**
             * @brief 
             *      Sets the positional soft limits in meters; the motor will 
             *      not intentionally leave the interval of values, [minpos, 
             *      maxpos].
             * 
             * This is meant to be used for setpoint checking.
             * @warning 
             *      This method is not currently implemented, do not use it 
             *      until a further release.
             * @param minpos
             *      Minimum position value in units::meter_t, must be less
             *      than maxpos
             * @param maxpos
             *      Maximum position values in units::meter_t, must be greater
             *      than minpos
             * @todo 
             *      Implement.
             */
            void SetPositionSoftLimits(units::meter_t /* minpos */, units::meter_t /* maxpos */) override;

            /**
             * @brief 
             *      Returns the state of the reverse limit switch.
             * 
             * This is used in TalonFXMotionCommand for checking whether the 
             * default/reverse/home limit switch is pressed.
             * @return 
             *      Boolean value, true = pressed, false = unpressed
             * @see MotorMotionCommand.h
             */
            bool IsRevLimitSwitchPressed() override;

            /**
             * @brief 
             *      Returns the state of the forward limit switch.
             * 
             * This is used in TalonFXMotionCommand for checking whether the 
             * non-default/forward/non-home limit switch is pressed.
             * @return 
             *      Boolean value, true = pressed, false = unpressed
             * @see MotorMotionCommand.h
             */
            bool IsFwdLimitSwitchPressed() override;

            /**
             * @brief 
             *      Stops the motor and resets the encoder to 0.
             * 
             * This is useful when powering on the robot and initializing 
             * subsystems.
             */
            void Reset() override;
    }; // class TalonFXMotion

} // namespace talonfx

} // namespace laser
