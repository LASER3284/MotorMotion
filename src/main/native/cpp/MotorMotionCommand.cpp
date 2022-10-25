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

#include "laser/MotorMotionCommand.h"

using namespace laser::commands;

MotorMotionCommand<class ErrorEnum, class MotorType>::MotorMotionCommand(MotorMotion<ErrorEnum, MotorType>* motionInstance, State action, units::second_t timeout = 0.0_s, double speed = 0.5) {
    motion = motionInstance;
    currentState = action;
    timer = new frc::Timer();
    maxHomeTime = timeout;
    revHomeSpeed = speed;
    fwdHomeSpeed = -speed;
}

MotorMotionCommand<class ErrorEnum, class MotorType>::~MotorMotionCommand() {
    delete timer;

    timer = nullptr;
}

void MotorMotionCommand<class ErrorEnum, class MotorType>::Initialize() {
    timer->Start();
    startTime = timer->Get();
}

void MotorMotionCommand<class ErrorEnum, class MotorType>::Execute() {
    // State machine
    switch(currentState) {
        case eIdle:
            // Stop the motor.
            motion->Stop();
            isFinished = true;
            break;

        case eHomeReverse:
            // Check to see if the home limit is pressed or if we have exceeded the maximum homing time.
            if ((motion->IsRevLimitSwitchPressed()) || ((maxHomeTime > 0.0_s) && (timer->Get() > (startTime + maxHomeTime)))) {
                // At the home limit switch, turn off the motor.
                motion->Set(0);
               
                // Reset the motor
                motion->Reset();
                // Move to idle.
                currentState = eIdle;
            } else {
                // Need to reach home switch
                motion->Set(revHomeSpeed);
            }
            break;

        case eHomeForward:
            // If the state is eHomingForward, the motor will slowly
            // move (forward) off the limit switch. Once the switch releases,
            // the motor will stop and the encoder will be reset.
            isFinished = false;

            // Check to see we are off the home limit switch or the homing timeout has been reached.
            if ((motion->IsFwdLimitSwitchPressed()) || ((maxHomeTime > 0.0_s) && (timer->Get() > (startTime + maxHomeTime)))) {
                // Reset the motor
                motion->Reset();
                // Set the state to eIdle.
                currentState = eIdle;
            } else {
                // Need to reach forward limit switch
                motion->Set(fwdHomeSpeed);
            }
            break;

        case eManualForward:
            if (!motion->IsFwdLimitSwitchPressed()) {
                // Manually move position forward.
                motion->Set(fwdHomeSpeed);
            } else {
                // Stop the motor
                motion->Set(0);
                // Change the state to eIdle.
                currentState = eIdle;
            }
            break;

        case eManualReverse:
            if (!motion->IsRevLimitSwitchPressed()) {
                // Manually move position backwards.
                motion->Set(revHomeSpeed);
            } else {
                // Stop the motor
                motion->Set(0);
                // Change the state to eIdle.
                currentState = eIdle;
            }
            break;

        default:
            currentState = eIdle;
            break;
    } // switch (currentState)
}

void MotorMotionCommand<class ErrorEnum, class MotorType>::End(bool interrupted) {
    if (interrupted)
        motion->Set(0);
    return;
}

bool MotorMotionCommand<class ErrorEnum, class MotorType>::IsFinished() {
    return isFinished;
}