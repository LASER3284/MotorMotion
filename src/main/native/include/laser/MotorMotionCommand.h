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
 * @file MotorMotionCommand.h
 * @brief 
 *      This file contains the Command structure classes for command-based 
 *      programming.
 */
#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <rev/CANSparkMax.h>
#include "laser/MotorMotion.h"
////////////////////////////////////////////////////////////////////////////////

namespace laser {

/**
 * @brief 
 *      This namespace contains command based implementation of MotorMotion 
 *      using a state machine.
 * 
 * This is used when using limit switches with a MotorMotion derived class, such
 * as for a turret, elevator, etc. It can be configured with with a homing 
 * timeout and a desired percent output on the input shaft. 
 */
namespace commands {
    /** 
     * @enum State
     * @brief 
     *      Motion states/commands.
     * 
     * These are used by the state machine to determine what to do. The instance
     * of MotorMotionCommand will use this to determine what action it should be
     * performing.
     */
    enum State {
        /** Do nothing */
        eIdle,
        /** Go towards reverse limit switch */
        eHomeReverse,
        /** Go towards forward limit switch */  
        eHomeForward,   
        /** Manual go towards forward limit switch */
        eManualForward, 
        /** Manual go towards reverse limit switch */
        eManualReverse  
    }; // enum State

    /**
     * @class MotorMotionCommand MotorMotionCommand.h laser/MotorMotionCommand.h
     * @brief 
     *      This class is used for controlling a MotorMotion derived class when 
     *      using limit switches.
     * 
     * It operates through a state machine, which is configured using the 
     * constructor and the State enum. This can be used to trigger an event 
     * whenever the command has finished.
     * @see State
     */
    template <typename ErrorEnum, class MotorType>
    class MotorMotionCommand : public frc2::CommandHelper<frc2::CommandBase, MotorMotionCommand<ErrorEnum, MotorType>> {
        public:
            /**
             * @brief 
             *      MotorMotionCommand constructor; initializes the command 
             *      structure. When the timeout is 0, there will be no timeout
             *      condition.
             * 
             * This class uses a state machine, and the desired action is 
             * specified through the constructor. This can be useful for doing 
             * an action relating to the limit switches.
             * @param motionInstance
             *      MotorMotion object pointer used for controlling the actual
             *      motor
             * @warning 
             *      The motionInstance is specified the way it is so that any 
             *      MotorMotion derived class can be used; this does not mean 
             *      that you can instantiate MotorMotion.
             * @param action
             *      State enum type that specifies what the command will 
             *      actually be doing with the motor 
             * @param timeout
             *      Seconds that the motor is allowed to use to home in 
             *      units::second_t (default 0, no timeout)
             * @param speed
             *      The maximum homing speed (in percent of max power) of the 
             *      motor (default 0.5, half power); note that this parameter
             *      sets the reverse (going home) speed to itself and the 
             *      forward (leaving home) speed to the negation of itself
             */
            MotorMotionCommand(MotorMotion<ErrorEnum, MotorType>* /* motionInstance */, State /* action */, units::second_t = 0.0_s /* timeout */, double = 0.5 /* speed */);

            /**
             * @brief 
             *      MotorMotionCommand destructor; deletes pointers to prevent
             *      memory leaks, with the exception of the pointer passed to
             *      the constructor.
             */
            ~MotorMotionCommand();

            /**
             * @brief 
             *      Initializes the MotorMotionCommand immediately before 
             *      execution.
             */
            void Initialize() override;

            /**
             * @brief 
             *      Main command method that will perform the action specified
             *      in the constructor; it contains a state machine based on 
             *      the given action.
             */
            void Execute() override;

            /**
             * @brief 
             *      Finalizes the command so that nothing is left over to cause
             *      issues.
             * @param interrupted
             *      Determines if the Execute function was interrupted or if it
             *      ended naturally upon IsFinished returning true
             */
            void End(bool /* interrupted */) override;

            /**
             * @brief 
             *      Returns a bool specifying whether the Execute command is
             *      finished
             * @return 
             *      A bool that, when true, the command finished normally; when false, the
             *      command is still executing
             */
            bool IsFinished() override;
        
        protected:
            /** @brief State enum type containing the action to be executed by the command */
            State currentState = eIdle;
            /** @brief MotorMotion<...> object pointer to control the physical motor using common commands */
            MotorMotion<ErrorEnum, MotorType>* motion;
            /** @brief frc::Timer object pointer for comparing timestamps against maxHomeTime */
            frc::Timer* timer;

            /** @brief A boolean value returned by IsFinished(); note the difference in capitalization */
            bool isFinished = false;

            /** @brief A units::second_t type that contains the start time from the timer */
            units::second_t startTime;
            /** @brief A units::second_t type that contains the max allowed time to home on the limit switches */
            units::second_t maxHomeTime;
            /** @brief A percentage of how fast it should be moving towards home switch */
            double revHomeSpeed;
            /** @brief A percentage of how fast it should be moving away from home switch */
            double fwdHomeSpeed;
    }; // class MotorMotionCommand

    /** @brief A typedef of MotorMotionCommand<...> specifically for TalonFXMotion */
    typedef MotorMotionCommand<ctre::phoenix::ErrorCode, ctre::phoenix::motorcontrol::can::WPI_TalonFX> TalonFXMotionCommand;
    /** @brief A typedef of MotorMotionCommand<...> specifically for SparkMaxMotion */
    typedef MotorMotionCommand<rev::REVLibError, rev::CANSparkMax> SparkMaxMotionCommand;

} // namespace commands

} // namespace laser
