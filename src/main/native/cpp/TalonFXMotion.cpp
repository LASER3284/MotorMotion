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

#include "laser/TalonFXMotion.h"

using namespace laser::talonfx;
////////////////////////////////////////////////////////////////////////////////

TalonFXMotion::TalonFXMotion(int devID, double ratio = 1.0, units::meter_t diameter = 1.0_m) {
    deviceID = devID;
    gearing = ratio;
    wheelDiameter = diameter;
    motor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(deviceID);

    // A setpoint must be specified before one can be used within the class
    setpointType = eNone;

    // Limit switches must be configured, otherwise they won't be used
    isFwdLimitSwitchNO = true;
    isRevLimitSwitchNO = true;

    // Reset the motor
    Reset();
}

TalonFXMotion::~TalonFXMotion() {
    delete motor;

    motor = nullptr;
}

void TalonFXMotion::SetSetpoint(units::meter_t position) {
    positionSetpoint = position;

    // Control through position
    // meters -> revolutions of output shaft -> revolutions of Falcon shaft -> 
    // encoder counts
    motor->Set(
        ctre::phoenix::motorcontrol::ControlMode::Position,
        (double)positionSetpoint / ((double)wheelDiameter * M_PI) * gearing * defaults::countsPerRev
    );

    setpointType = ePosition;
}

void TalonFXMotion::SetSetpoint(units::meters_per_second_t lvelocity) {
    velocitySetpoint = lvelocity;

    // Control through linear velocity
    // meters per second -> revs per sec, output shaft -> revs per sec, input 
    // shaft -> counts per sec -> counts per 100ms
    motor->Set(
        ctre::phoenix::motorcontrol::ControlMode::Velocity,
        (double)velocitySetpoint / ((double)wheelDiameter * M_PI) * gearing * defaults::countsPerRev / 10
    );

    setpointType = eLinearVelocity;
}

void TalonFXMotion::SetSetpoint(units::radians_per_second_t avelocity) {
    avelSetpoint = avelocity;

    // Control through linear velocity
    // rad per second -> revs per sec, output shaft -> revs per sec, input shaft -> counts per sec -> counts per 100ms
    motor->Set(
        ctre::phoenix::motorcontrol::ControlMode::Velocity,
        (double)avelSetpoint / (2 * M_PI) * gearing * defaults::countsPerRev / 10
    );

    setpointType = eAngularVelocity;
}

void TalonFXMotion::SetMotorVoltage(units::volt_t voltage) {
    motor->SetVoltage(voltage);
    motor->Feed();
}

void TalonFXMotion::Stop() {
    // Stop the motor.
    motor->Set(0.000);
}

void TalonFXMotion::SetTolerance(units::meter_t tolerance) {
    // meters -> rotations -> encoder counts
    motor->ConfigAllowableClosedloopError(
        0,
        (double)tolerance / ((double)wheelDiameter * M_PI) * defaults::countsPerRev 
    );

    // Set the member variable.
    positionTolerance = tolerance;
}

void TalonFXMotion::SetTolerance(units::meters_per_second_t tolerance) {
    // meters per sec -> rotations per sec -> encoder counts per sec -> encoder
    // counts per 100ms
    motor->ConfigAllowableClosedloopError(
        0,
        (double)tolerance / ((double)wheelDiameter * M_PI) * defaults::countsPerRev / 10
    );

    // Set the member variable.
    velocityTolerance = tolerance;
}

void TalonFXMotion::SetTolerance(units::radians_per_second_t tolerance) {
    // radians per sec -> rotations per sec -> encoder counts per sec -> 
    // encoder counts per 100ms
    motor->ConfigAllowableClosedloopError(
        0,
        (double)tolerance / (2 * M_PI) * defaults::countsPerRev / 10
    );

    // Set the member variable.
    avelTolerance = tolerance;
}

units::meter_t TalonFXMotion::GetPositionTolerance() {
    return positionTolerance;
}

units::meters_per_second_t TalonFXMotion::GetVelocityTolerance() {
    return velocityTolerance;
}

units::radians_per_second_t TalonFXMotion::GetAngularVelocityTolerance() {
    return avelTolerance;
}

void TalonFXMotion::ConfigLimitSwitches(bool isFwdNO, bool isRevNO) {
    // Set the member variables.
    isFwdLimitSwitchNO = isFwdNO;
    isRevLimitSwitchNO = isRevNO;

    // Set the internal lim. sw. configs
    motor->ConfigForwardLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
        (isFwdLimitSwitchNO ? ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_NormallyOpen :
        ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_NormallyClosed)
    );
    motor->ConfigReverseLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
        (isRevLimitSwitchNO ? ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_NormallyOpen :
        ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_NormallyClosed)
    );
}

void TalonFXMotion::SetAccumIZone(double _izone) {
    // Set the member variable.
    izone = _izone;

    // revolutions, output shaft -> revolutions, input shaft -> encoder ticks
    motor->Config_IntegralZone(0, izone / gearing * 2048);
}

bool TalonFXMotion::IsFwdLimitSwitchPressed() {
    return (
        (isFwdLimitSwitchNO && motor->GetSensorCollection().IsFwdLimitSwitchClosed()) ||
        (!isFwdLimitSwitchNO && !motor->GetSensorCollection().IsFwdLimitSwitchClosed())
    );
}

bool TalonFXMotion::IsRevLimitSwitchPressed() {
    return (
        (isRevLimitSwitchNO && motor->GetSensorCollection().IsRevLimitSwitchClosed()) ||
        (!isRevLimitSwitchNO && !motor->GetSensorCollection().IsRevLimitSwitchClosed())
    );
}

void TalonFXMotion::Reset() {
    Stop();
    // Reset the encoder count to zero.
    motor->SetSelectedSensorPosition(0);
}

void TalonFXMotion::SetOpenRampRate(units::second_t time) {
    motor->ConfigOpenloopRamp((double)time);
}

void TalonFXMotion::SetClosedRampRate(units::second_t time) {
    motor->ConfigClosedloopRamp((double)time);
}

units::meter_t TalonFXMotion::GetActualPosition() {
    units::meter_t actual = 0.0_m;

    // Sensor units -> revolutions, input shaft -> revolutions, output shaft ->
    // meters, distance of wheel
    actual = motor->GetSelectedSensorPosition() / defaults::countsPerRev / gearing * (wheelDiameter * M_PI);

    return actual;
}

units::meters_per_second_t TalonFXMotion::GetActualVelocity() {
    units::meters_per_second_t actual = 0.0_mps;

    // Sensor units per 100ms -> sensor units per second -> revolutions per 
    // sec, input shaft -> rev per sec, output shaft -> m/s, wheel speed
    actual = motor->GetSelectedSensorVelocity() * 10 / defaults::countsPerRev / gearing * (wheelDiameter * M_PI) / 1.0_s;

    return actual;
}

units::radians_per_second_t TalonFXMotion::GetActualAngularVelocity() {
    units::radians_per_second_t actual = units::radians_per_second_t(0.0);

    // Sensor units per 100ms -> sensor units per sec -> revs per sec, input 
    // shaft -> revs per sec, output shaft -> rad/s, output shaft
    actual = units::radians_per_second_t(
        motor->GetSelectedSensorVelocity() * 10 / defaults::countsPerRev / gearing * (2 * M_PI)
    );

    return actual;
}

void TalonFXMotion::SetPIDValues(
    double proportional,
    double integral, 
    double derivative, 
    double feedforward
) {
    // Set PID values for either position or velocity
    switch (setpointType) {
        case eNone:
            break;

        case ePosition:
            positionProportional = proportional;
            positionIntegral = integral;
            positionDerivative = derivative;
            positionFeedForward = feedforward;

            motor->Config_kP(0, positionProportional);
            motor->Config_kI(0, positionIntegral);
            motor->Config_kD(0, positionDerivative);
            motor->Config_kF(0, positionFeedForward);

            break;
        
        case eLinearVelocity:
            velocityProportional = proportional;
            velocityIntegral = integral;
            velocityDerivative = derivative;
            velocityFeedForward = feedforward;

            motor->Config_kP(0, velocityProportional);
            motor->Config_kI(0, velocityIntegral);
            motor->Config_kD(0, velocityDerivative);
            motor->Config_kF(0, velocityFeedForward);

            break;

        case eAngularVelocity:
            avelProportional = proportional;
            avelIntegral = integral;
            avelDerivative = derivative;
            avelFeedForward = feedforward;

            motor->Config_kP(0, avelProportional);
            motor->Config_kI(0, avelIntegral);
            motor->Config_kD(0, avelDerivative);
            motor->Config_kF(0, avelFeedForward);

            break;

        default:
            break;
    }
}

void TalonFXMotion::SetMotorInverted(bool isInverted) {
    // Whenever a positive input is sent to the motor controller, the output 
    // will be reversed/negated
    motor->SetInverted(isInverted);
}

ctre::phoenix::ErrorCode TalonFXMotion::ConfigCurrentLimit(units::ampere_t amps) {
    // good lord, please forgive me for my sins
    // TODO: make this better
    ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration config;
    if (amps == 0_A) {
        config = ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(false, 0, 0, 0);
    } else {
        config = ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, (double)amps, 0, 0);
    }

    return motor->ConfigSupplyCurrentLimit(config);
}

