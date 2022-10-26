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
 * TODO: Refactor the whole file to match MotorMotion
 */
#include "laser/SparkMaxMotion.h"
////////////////////////////////////////////////////////////////////////////////

CSparkMotion::CSparkMotion(int nDeviceID)
{
    m_nDeviceID = nDeviceID;
    // Create the object pointers.
    m_pMotor						= new CANSparkMax(nDeviceID, CANSparkMax::MotorType::kBrushless);
    m_pPIDController                = new SparkMaxPIDController(m_pMotor->GetPIDController());
    m_pTimer						= new Timer();

    // Initialize member variables.
    m_nCurrentState					= eIdle;
    m_bReady						= true;
    m_bFwdLimitSwitchNormallyOpen	= true;
    m_bRevLimitSwitchNormallyOpen	= true;
    m_bHomingComplete				= false;
    m_bBackOffHome					= true;
    m_bMotionMagic					= false;
    m_bUsePosition					= true;
    m_dSetpoint						= 0.000;
    m_nPulsesPerRev					= nDefaultSparkMotionPulsesPerRev;
    m_dTimeUnitInterval				= dDefaultSparkMotionTimeUnitInterval;
    m_dRevsPerUnit					= dDefaultSparkMotionRevsPerUnit;
    m_dFwdMoveSpeed					= dDefualtSparkMotionManualFwdSpeed;
    m_dRevMoveSpeed					= dDefualtSparkMotionManualRevSpeed;
    m_dFwdHomeSpeed					= dDefaultSparkMotionFwdHomeSpeed;
    m_dRevHomeSpeed					= dDefaultSparkMotionRevHomeSpeed;
    m_dTolerance					= dDefaultSparkMotionTolerance;
    m_dLowerPositionSoftLimit		= dDefaultSparkMotionLowerPositionSoftLimit;
    m_dUpperPositionSoftLimit		= dDefaultSparkMotionUpperPositionSoftLimit;
    m_dLowerVelocitySoftLimit		= dDefaultSparkMotionLowerVelocitySoftLimit;
    m_dUpperVelocitySoftLimit		= dDefaultSparkMotionUpperVelocitySoftLimit;
    m_dIZone						= dDefaultSparkMotionIZone;
    m_dMaxHomingTime				= dDefaultSparkMotionMaxHomingTime;
    m_dMaxFindingTime				= dDefaultSparkMotionMaxFindingTime;
    m_dHomingStartTime				= 0.000;
    m_dFindingStartTime				= 0.000;

    // Reset the encoder count to zero.
    ResetEncoderPosition();
    // Set the motor as positive.
    SetMotorInverted(false);
    // Set up the nominal motor output for both directions.
    SetNominalOutputVoltage(0.000, 0.000);
    // Set the peak (maximum) motor output for both directions.
    SetPeakOutputPercent(1.000, -1.000);
    // Set the tolerance.
    SetTolerance(m_dTolerance);
    // Set the PID and feed forward values.
    SetPIDValues(dDefaultSparkMotionProportional, dDefaultSparkMotionIntegral, dDefaultSparkMotionDerivative, dDefaultSparkMotionFeedForward);
    // Stop the motor.
    Stop();
    // Set the neutral mode to brake.
    m_pMotor->SetIdleMode(CANSparkMax::IdleMode::kBrake);
    // Disable both forward and reverse limit switches.
    m_pMotor->GetForwardLimitSwitch(SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(false);	
    m_pMotor->GetReverseLimitSwitch(SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(false);	
    // Set acceleration (seconds from neutral to full output).
    SetOpenLoopRampRate(dDefaultSparkMotionVoltageRampRate);
    SetClosedLoopRampRate(dDefaultSparkMotionVoltageRampRate);
    // Clear the sticky faults in memory.
    ClearStickyFaults();
    // Set the Integral Zone. Accumulated integral is reset to zero when the error exceeds this value.
    SetAccumIZone(m_dIZone);
    // Clear the sticky faults in memory.
    ClearStickyFaults();

    // Start the timer.
    m_pTimer->Start();
}

CSparkMotion::~CSparkMotion()
{
    // Delete our object pointers.
    delete	m_pMotor;
    delete	m_pTimer;

    // Set the objects to NULL.
    m_pMotor	= nullptr;
    m_pTimer	= nullptr;
}

void CSparkMotion::Tick()
{
    // State machine
    switch(m_nCurrentState)
    {
        case eIdle :
            // Stop the motor.
            m_pPIDController->SetReference(0, CANSparkMax::ControlType::kVoltage);
            m_bReady = true;
            break;

        case eHomingReverse :
            // If the state is eHomingReverse, the motor will move toward
            // the home switch, and then turn off and go to eHomingForward.
            m_bReady = false;

            // Check to see if the home limit is pressed or if we have exceeded the maximum homing time.
            if ((IsRevLimitSwitchPressed()) ||
                ((m_dMaxHomingTime > 0.000) && ((double)m_pTimer->Get() > (m_dHomingStartTime + m_dMaxHomingTime))))
            {
                // At the home limit switch, turn off the motor.
                m_pPIDController->SetReference(0.000, CANSparkMax::ControlType::kVoltage);
                if (m_bBackOffHome)
                {
                    // Set the state to eHomingForward.
                    m_nCurrentState = eHomingForward;
                }
                else
                {
                    // Reset the encoder to zero.
                    // Stop the motor and change the control mode to position.
                    m_pPIDController->SetReference(0, CANSparkMax::ControlType::kVoltage);
                    // Set flag that homing is complete.
                    m_bHomingComplete = true;
                    // Move to idle.
                    m_nCurrentState = eIdle;
                }
            }
            else
            {
                // Not yet at the home limit switch, keep moving.
                m_pPIDController->SetReference(m_dRevHomeSpeed, CANSparkMax::ControlType::kDutyCycle);
            }
            break;

        case eHomingForward :
            // If the state is eHomingForward, the motor will slowly
            // move (forward) off the limit switch. Once the switch releases,
            // the motor will stop and the encoder will be reset.
            m_bReady = false;

            // Check to see we are off the home limit switch or the homing timeout has been reached.
            if ((!IsRevLimitSwitchPressed()) ||
                ((m_dMaxHomingTime > 0.000) && ((double)m_pTimer->Get() > (m_dHomingStartTime + m_dMaxHomingTime))))
            {
                // Stop the motor and change the control mode to position.
                m_pPIDController->SetReference(0, CANSparkMax::ControlType::kPosition);
                // Set flag that homing is complete.
                m_bHomingComplete = true;
                // Set the state to eIdle.
                m_nCurrentState = eIdle;
            }
            else
            {
                // Still on the home limit switch, keep moving.
                m_pPIDController->SetReference(m_dFwdHomeSpeed, CANSparkMax::ControlType::kDutyCycle);
            }
            break;

        case eFinding :
            // If the state is eFinding, the motor will continue until
            // the PID reaches the target or until the limit switch in
            // the direction of travel is pressed. The state then becomes idle.
            m_bReady = false;
            // Check to see if position is within tolerance or limit switch
            // is activated in direction of travel.
            if (IsAtSetpoint() ||
               (((GetSetpoint() > GetActual()) && IsFwdLimitSwitchPressed()) ||
                ((GetSetpoint() < GetActual()) && IsRevLimitSwitchPressed()) ||
                ((m_dMaxFindingTime > 0.000) && ((double)m_pTimer->Get() > (m_dFindingStartTime + m_dMaxFindingTime)))))
            {
                // Stop the motor and set the current state to eIdle.
                Stop();
            }
            break;

        case eManualForward :
            if (!IsFwdLimitSwitchPressed())
            {
                // Manually move position forward.
                m_pPIDController->SetReference(m_dFwdMoveSpeed, CANSparkMax::ControlType::kDutyCycle);
                m_bReady = false;
            }
            else
            {
                // Change the state to eIdle.
                SetState(eIdle);
                m_bReady = true;
            }
            break;

        case eManualReverse :
            if (!IsRevLimitSwitchPressed())
            {
                // Manually move position backwards.
                m_pPIDController->SetReference(m_dRevMoveSpeed, CANSparkMax::ControlType::kDutyCycle);
                m_bReady = false;
            }
            else
            {
                // Change the state to eIdle.
                SetState(eIdle);
                m_bReady = true;
            }
            break;

        default :
            break;
    }
}

void CSparkMotion::SetSetpoint(double dSetpoint, bool bUsePosition)
{
    // Set the bUsePosition member variable.
    m_bUsePosition = bUsePosition;

    // Use either position or velocity setpoints.
    if (bUsePosition)
    {
        // Clamp the new setpoint within soft limits.
        if (dSetpoint > m_dUpperPositionSoftLimit)
        {
            dSetpoint = m_dUpperPositionSoftLimit;
        }
        else
        {
            if (dSetpoint < m_dLowerPositionSoftLimit)
            {
                dSetpoint = m_dLowerPositionSoftLimit;
            }
        }

        // Set the dSetpoint member variable so other methods can access it.
        m_dSetpoint = dSetpoint;

        // Set the motor to the desired position.
        if (m_bMotionMagic)
        {
            m_pPIDController->SetReference(dSetpoint * m_dRevsPerUnit * m_nPulsesPerRev, CANSparkMax::ControlType::kSmartMotion);
        }
        else
        {
            m_pPIDController->SetReference(dSetpoint * m_dRevsPerUnit * m_nPulsesPerRev, CANSparkMaxLowLevel::ControlType::kPosition);
        }
    }
    else
    {
        // Clamp the new setpoint within soft limits.
        if (dSetpoint > m_dUpperVelocitySoftLimit)
        {
            dSetpoint = m_dUpperVelocitySoftLimit;
        }
        else
        {
            if (dSetpoint < m_dLowerVelocitySoftLimit)
            {
                dSetpoint = m_dLowerVelocitySoftLimit;
            }
        }

        // Set the dSetpoint member variable so other methods can access it.
        m_dSetpoint = dSetpoint;

        // Set the motor to the desired position.
        if (m_bMotionMagic)
        {
            m_pPIDController->SetReference(dSetpoint * (84 /8 * m_nPulsesPerRev), CANSparkMax::ControlType::kSmartVelocity);
        }
        else
        {
            m_pPIDController->SetReference(dSetpoint * (84 /8 * m_nPulsesPerRev), CANSparkMax::ControlType::kVelocity);
        }
    }

    // Prints can slow down the processing time of the RoboRIO, so these are for debugging.
//	printf("CSparkMotion::SetSetpoint - Setpoint = %7.3f\n", dSetpoint);
//	printf("CSparkMotion::SetSetpoint - Revs Per Unit = %7.3f\n", m_dRevsPerUnit);

    // Set the state to eFinding.
    m_nCurrentState = eFinding;
}

double CSparkMotion::GetSetpoint()
{
    return m_dSetpoint;
}

void CSparkMotion::StartHoming()
{
    // Stop the motor and set the control mode for percent output.
    m_pPIDController->SetReference(0, CANSparkMax::ControlType::kDutyCycle);

    // Get the homing start time.
    m_dHomingStartTime = (double)m_pTimer->Get();

    // Set flag that homing is not complete.
    m_bHomingComplete = false;

    // Set the current state to eHomingReverse.
    m_nCurrentState = eHomingReverse;
}

void CSparkMotion::Stop()
{
    // Stop the motor.
    m_pPIDController->SetReference(0, CANSparkMax::ControlType::kDutyCycle);

    // Set the current state to eIdle.
    m_nCurrentState = eIdle;
}

void CSparkMotion::SetTolerance(double dValue)
{
    // Set the member variable.
    m_dTolerance = dValue;

    // Set the allowed error for the PID. This is in quadrature pulses.
    //TODO: Find equivalent for "non-Motion Magic" closed loop error.
    m_pPIDController->SetSmartMotionAllowedClosedLoopError(m_dTolerance * m_dRevsPerUnit * m_nPulsesPerRev);
}

double CSparkMotion::GetTolerance()
{
    return m_dTolerance;
}

void CSparkMotion::SetPositionSoftLimits(double dMinValue, double dMaxValue)
{
    // Set the member variables.
    m_dLowerPositionSoftLimit	= dMinValue;
    m_dUpperPositionSoftLimit	= dMaxValue;
}

void CSparkMotion::SetVelocitySoftLimits(double dMinValue, double dMaxValue)
{
    // Set the member variables.
    m_dLowerVelocitySoftLimit	= dMinValue;
    m_dUpperVelocitySoftLimit	= dMaxValue;
}

void CSparkMotion::ConfigLimitSwitches(bool bFwdLimit, bool bRevLimit)
{
    // Set the member variables.
    m_bFwdLimitSwitchNormallyOpen = bFwdLimit;
    m_bRevLimitSwitchNormallyOpen = bRevLimit;

    m_pMotor->GetForwardLimitSwitch(bFwdLimit ?
                                    SparkMaxLimitSwitch::Type::kNormallyOpen : 
                                    SparkMaxLimitSwitch::Type::kNormallyClosed).EnableLimitSwitch(true);	
    m_pMotor->GetReverseLimitSwitch(bRevLimit ?
                                    SparkMaxLimitSwitch::Type::kNormallyOpen : 
                                    SparkMaxLimitSwitch::Type::kNormallyClosed).EnableLimitSwitch(true);	
}

void CSparkMotion::SetAccumIZone(double dIZone)
{
    // Set the member variable.
    m_dIZone = dIZone;

    // Set the Integral Zone. Accumulated integral is reset to zero when the error exceeds this value.
    m_pPIDController->SetIZone(m_dIZone * m_dRevsPerUnit * m_nPulsesPerRev);
}

bool CSparkMotion::IsFwdLimitSwitchPressed()
{
    return ((m_bFwdLimitSwitchNormallyOpen && m_pMotor->GetForwardLimitSwitch(m_bFwdLimitSwitchNormallyOpen ?
                                              SparkMaxLimitSwitch::Type::kNormallyOpen : 
                                              SparkMaxLimitSwitch::Type::kNormallyClosed).Get()) ||
           (!m_bFwdLimitSwitchNormallyOpen && !m_pMotor->GetForwardLimitSwitch(m_bFwdLimitSwitchNormallyOpen ?
                                              SparkMaxLimitSwitch::Type::kNormallyOpen : 
                                              SparkMaxLimitSwitch::Type::kNormallyClosed).Get()));
}

bool CSparkMotion::IsRevLimitSwitchPressed()
{
        return ((m_bRevLimitSwitchNormallyOpen && m_pMotor->GetReverseLimitSwitch(m_bRevLimitSwitchNormallyOpen ?
                                              SparkMaxLimitSwitch::Type::kNormallyOpen : 
                                              SparkMaxLimitSwitch::Type::kNormallyClosed).Get()) ||
           (!m_bRevLimitSwitchNormallyOpen && !m_pMotor->GetReverseLimitSwitch(m_bRevLimitSwitchNormallyOpen ?
                                              SparkMaxLimitSwitch::Type::kNormallyOpen : 
                                              SparkMaxLimitSwitch::Type::kNormallyClosed).Get()));
}

bool CSparkMotion::IsAtSetpoint()
{
    return (((fabs(GetSetpoint() - GetActual())) < m_dTolerance) && (fabs(m_pMotor->GetBusVoltage()) < 1.000));
}

void CSparkMotion::ResetEncoderPosition()
{
    // Reset the encoder count to zero.
//TODO: Figure this out!
//	m_pMotor->SetSelectedSensorPosition(0);
}

void CSparkMotion::SetPeakOutputPercent(double dMaxFwdOutput, double dMaxRevOutput)
{
    m_pPIDController->SetOutputRange(dMaxFwdOutput, dMaxRevOutput);
}

void CSparkMotion::SetNominalOutputVoltage(double dNominalFwdOutput, double dNominalRevOutput)
{
//	m_pMotor->ConfigNominalOutputForward(dNominalFwdOutput);
//	m_pMotor->ConfigNominalOutputReverse(dNominalRevOutput);
}

void CSparkMotion::SetOpenLoopRampRate(double dOpenLoopRampRate)
{
    m_pMotor->SetOpenLoopRampRate(dOpenLoopRampRate);
}

void CSparkMotion::SetClosedLoopRampRate(double dClosedLoopRampRate)
{
    m_pMotor->SetClosedLoopRampRate(dClosedLoopRampRate);
}

void CSparkMotion::SetMotorNeutralMode(int nMode)
{
    m_pMotor->SetIdleMode((nMode == 1) ? CANSparkMax::IdleMode::kCoast : CANSparkMax::IdleMode::kBrake);
}

double CSparkMotion::GetActual()
{
    // Create instance variables.
    double dActual = 0.000;

    if (m_bUsePosition)
    {
        dActual = (m_pMotor->GetEncoder().GetPosition() / m_dRevsPerUnit / m_nPulsesPerRev);
    }
    else
    {
        dActual = (m_pMotor->GetEncoder().GetVelocity() / (84 / 8 * m_nPulsesPerRev) /** m_dTimeUnitInterval*/);
    }

    return dActual;
}

void CSparkMotion::SetHomeSpeeds(double dFwdSpeed, double dRevSpeed)
{
    m_dFwdHomeSpeed = dFwdSpeed;
    m_dRevHomeSpeed = dRevSpeed;
}

void CSparkMotion::SetPulsesPerRev(int nPPR)
{
    m_nPulsesPerRev = nPPR;
}

void CSparkMotion::SetRevsPerUnit(double dRPU)
{
    m_dRevsPerUnit = dRPU;
}

void CSparkMotion::SetPIDValues(double dProportional, double dIntegral, double dDerivative, double dFeedForward)
{
    m_pPIDController->SetP(dProportional);
    m_pPIDController->SetI(dIntegral);
    m_pPIDController->SetD(dDerivative);
    m_pPIDController->SetFF(dFeedForward);
}

void CSparkMotion::SetMotorInverted(bool bInverted)
{
    m_pMotor->SetInverted(bInverted);
}

void CSparkMotion::ClearStickyFaults()
{
    m_pMotor->ClearFaults();
}

void CSparkMotion::SetManualSpeed(double dForward, double dReverse)
{
    m_dFwdMoveSpeed = dForward;
    m_dRevMoveSpeed = dReverse;
}

void CSparkMotion::SetAcceleration(double dRPS)
{
    m_pPIDController->SetSmartMotionMaxAccel(dRPS);
}

void CSparkMotion::SetCruiseRPM(double dRPM)
{
    m_pPIDController->SetSmartMotionMaxVelocity(dRPM);
}
///////////////////////////////////////////////////////////////////////////////