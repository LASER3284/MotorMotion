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

using namespace frc;
using namespace units;
using namespace ctre::phoenix::motorcontrol;
///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
	Description:	FalconMotion Constructor.
	Arguments:		int nDeviceID - CAN Bus Device ID
	Derived From:	Nothing
******************************************************************************/
FalconMotion::FalconMotion(int nDeviceID) {
	// Create the object pointers.
	m_pMotor						= new WPI_TalonFX(nDeviceID);
	m_pTimer						= new Timer();

	// Initialize member variables.
	m_nCurrentState					= eIdle;
	isReady						= true;
	isFwdLimitSwitchNormallyOpen	= true;
	isRevLimitSwitchNormallyOpen	= true;
	isHomingComplete				= false;
	isBackOffHome					= true;
	isMotionMagic					= false;
	isUsePosition					= true;
	m_dSetpoint						= 0.000;
	m_nDeviceID 					= nDeviceID;
	m_nPulsesPerRev					= nDefaultFalconMotionPulsesPerRev;
	m_dRevsPerUnit					= dDefaultFalconMotionRevsPerUnit;
	m_dTimeUnitInterval				= dDefaultFalconMotionTimeUnitInterval;
	m_dFwdMoveSpeed					= dDefualtFalconMotionManualFwdSpeed;
	m_dRevMoveSpeed					= dDefualtFalconMotionManualRevSpeed;
	m_dFwdHomeSpeed					= dDefaultFalconMotionFwdHomeSpeed;
	m_dRevHomeSpeed					= dDefaultFalconMotionRevHomeSpeed;
	m_dPositionProportional			= dDefaultFalconMotionPositionProportional;
	m_dPositionIntegral				= dDefaultFalconMotionPositionIntegral;
	m_dPositionDerivative			= dDefaultFalconMotionPositionDerivative;
	m_dVelocityProportional			= dDefaultFalconMotionVelocityProportional;
	m_dVelocityIntegral				= dDefaultFalconMotionVelocityIntegral;
	m_dVelocityDerivative			= dDefaultFalconMotionVelocityDerivative;
	m_dPositionTolerance			= dDefaultFalconMotionPositionTolerance;
	m_dVelocityTolerance			= dDefaultFalconMotionVelocityTolerance;
	m_dLowerPositionSoftLimit		= dDefaultFalconMotionLowerPositionSoftLimit;
	m_dUpperPositionSoftLimit		= dDefaultFalconMotionUpperPositionSoftLimit;
	m_dLowerVelocitySoftLimit		= dDefaultFalconMotionLowerVelocitySoftLimit;
	m_dUpperVelocitySoftLimit		= dDefaultFalconMotionUpperVelocitySoftLimit;
	m_dIZone						= dDefaultFalconMotionIZone;
	m_dMaxHomingTime				= dDefaultFalconMotionMaxHomingTime;
	m_dMaxFindingTime				= dDefaultFalconMotionMaxFindingTime;
	m_dHomingStartTime				= 0.000;
	m_dFindingStartTime				= 0.000;

	// Set up the feedback device for a quadrature encoder.
	m_pMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
	// Reset the encoder count to zero.
	ResetEncoderPosition();
	// Set the encoder and motor as both positive.
	SetMotorInverted(false);
	SetSensorInverted(false);
	// Set up the nominal motor output for both directions.
	SetNominalOutputVoltage(0.000, 0.000);
	// Set the peak (maximum) motor output for both directions.
	SetPeakOutputPercent(1.000, -1.000);
	// Set the tolerance for position and velocity PIDs.
	SetTolerance(m_dPositionTolerance);
	SetTolerance(m_dVelocityTolerance);
	// Set the PID and feed forward values for position.
	SetPIDValues(dDefaultFalconMotionPositionProportional, dDefaultFalconMotionPositionIntegral, dDefaultFalconMotionPositionDerivative, dDefaultFalconMotionFeedForward);
	// Stop the motor.
	Stop();
	// Set the neutral mode to brake.
	m_pMotor->SetNeutralMode(NeutralMode::Brake);
	// Disable both forward and reverse limit switches.
	m_pMotor->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_Disabled);
	m_pMotor->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_Disabled);
	// Set acceleration (seconds from neutral to full output).
	SetOpenLoopRampRate(dDefaultFalconMotionVoltageRampRate);
	SetClosedLoopRampRate(dDefaultFalconMotionVoltageRampRate);
	// Clear the sticky faults in memory.
	ClearStickyFaults();
	// Set the Integral Zone. Accumulated integral is reset to zero when the error exceeds this value.
	SetAccumIZone(m_dIZone);
	// Clear the sticky faults in memory.
	ClearStickyFaults();

	// Start the timer.
	m_pTimer->Start();
}

/******************************************************************************
	Description:	FalconMotion Destructor.
	Arguments:		None
	Derived From:	Nothing
******************************************************************************/
FalconMotion::~FalconMotion()
{
	// Delete our object pointers.
	delete	m_pMotor;
	delete	m_pTimer;

	// Set the objects to NULL.
	m_pMotor	= nullptr;
	m_pTimer	= nullptr;
}

/******************************************************************************
	Description:	Tick - main method that does functionality.
					Called each time through robot main loop to update state.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::Tick()
{
	// State machine
	switch(m_nCurrentState)
	{
		case eIdle :
			// Stop the motor.
			m_pMotor->Set(ControlMode::PercentOutput, 0.000);
			isReady = true;
			break;

		case eHomingReverse :
			// If the state is eHomingReverse, the motor will move toward
			// the home switch, and then turn off and go to eHomingForward.
			isReady = false;

			// Check to see if the home limit is pressed or if we have exceeded the maximum homing time.
			if ((IsRevLimitSwitchPressed()) ||
				((m_dMaxHomingTime > 0.000) && ((double)m_pTimer->Get() > (m_dHomingStartTime + m_dMaxHomingTime))))
			{
				// At the home limit switch, turn off the motor.
				m_pMotor->Set(ControlMode::PercentOutput, 0.00);
				if (isBackOffHome)
				{
					// Set the state to eHomingForward.
					m_nCurrentState = eHomingForward;
				}
				else
				{
					// Reset the encoder to zero.
					m_pMotor->SetSelectedSensorPosition(0);
					// Stop the motor and change the control mode to position.
					m_pMotor->Set(ControlMode::Position, 0.000);
					// Set flag that homing is complete.
					isHomingComplete = true;
					// Move to idle.
					m_nCurrentState = eIdle;
				}
			}
			break;

		case eHomingForward :
			// If the state is eHomingForward, the motor will slowly
			// move (forward) off the limit switch. Once the switch releases,
			// the motor will stop and the encoder will be reset.
			isReady = false;

			// Check to see we are off the home limit switch or the homing timeout has been reached.
			if ((!IsRevLimitSwitchPressed()) ||
				((m_dMaxHomingTime > 0.000) && ((double)m_pTimer->Get() > (m_dHomingStartTime + m_dMaxHomingTime))))
			{
				// Reset the encoder to zero.
				m_pMotor->SetSelectedSensorPosition(0);
				// Stop the motor and change the control mode to position.
				m_pMotor->Set(ControlMode::Position, 0.000);
				// Set flag that homing is complete.
				isHomingComplete = true;
				// Set the state to eIdle.
				m_nCurrentState = eIdle;
			}
			else
			{
				// Still on the home limit switch, keep moving.
				m_pMotor->Set(ControlMode::PercentOutput, m_dFwdHomeSpeed);
			}
			break;

		case eFinding :
			// If the state is eFinding, the motor will continue until
			// the PID reaches the target or until the limit switch in
			// the direction of travel is pressed. The state then becomes idle.
			isReady = false;
			// Check to see if position is within tolerance or limit switch
			// is activated in direction of travel.
			if (IsAtSetpoint() ||
				(((GetSetpoint() > GetActual(isUsePosition)) && IsFwdLimitSwitchPressed()) ||
				((GetSetpoint() < GetActual(isUsePosition)) && IsRevLimitSwitchPressed()) ||
				((m_dMaxFindingTime > 0.000) && ((double)m_pTimer->Get() > (m_dFindingStartTime + m_dMaxFindingTime)))))
			{
				// Only stop timer when using position.
				if (isUsePosition)
				{
					// Stop the motor and set the current state to eIdle.
					Stop();
				}				
			}	
			break;

		case eManualForward :
			if (!IsFwdLimitSwitchPressed())
			{
				// Manually move position forward.
				m_pMotor->Set(ControlMode::PercentOutput, m_dFwdMoveSpeed);
				isReady = false;
			}
			else
			{
				// Change the state to eIdle.
				SetState(eIdle);
				isReady = true;
			}
			break;

		case eManualReverse :
			if (!IsRevLimitSwitchPressed())
			{
				// Manually move position backwards.
				m_pMotor->Set(ControlMode::PercentOutput, m_dRevMoveSpeed);
				isReady = false;
			}
			else
			{
				// Change the state to eIdle.
				SetState(eIdle);
				isReady = true;
			}
			break;

		default :
			break;
	}
}

/******************************************************************************
	Description:	SetSetpoint - Sets the position for the motor.
	Arguments:	 	dSetpoint - The position to move to in desired units.
					bUsePosition - Select position or velocity setpoint.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetSetpoint(double dSetpoint, bool bUsePosition)
{
	// Set the bUsePosition member variable.
	isUsePosition = bUsePosition;

	// Get current time.
	m_dFindingStartTime = (double)m_pTimer->Get();

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
		if (isMotionMagic)
		{
			m_pMotor->Set(ControlMode::MotionMagic, dSetpoint * m_dRevsPerUnit * m_nPulsesPerRev);
		}
		else
		{
			m_pMotor->Set(ControlMode::Position, dSetpoint * m_dRevsPerUnit * m_nPulsesPerRev);
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
		m_pMotor->Set(ControlMode::Velocity, (dSetpoint * m_dRevsPerUnit * m_nPulsesPerRev) / m_dTimeUnitInterval);
	}

	// Set the state to eFinding.
	m_nCurrentState = eFinding;
}

/******************************************************************************
	Description:	SetMotorVoltage - Set the desired motor voltage.
	Arguments:	 	dVoltage - The motor voltage.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetMotorVoltage(double dVoltage)
{
	m_pMotor->SetVoltage(volt_t(dVoltage));
	m_pMotor->Feed();
}

/******************************************************************************
	Description:	GetSetpoint - Returns the current setpoint of the motor's
					PID in desired units of measure.
	Arguments:	 	None
	Returns: 		The setpoint of the motor's PID in desired units of measure.
******************************************************************************/
double FalconMotion::GetSetpoint()
{
	return m_dSetpoint;
}

/******************************************************************************
	Description:	StartHoming - Initializes the homing sequence.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::StartHoming()
{
	// Stop the motor and set the control mode for percent output.
	m_pMotor->Set(ControlMode::PercentOutput, 0.000);

	// Get the homing start time.
	m_dHomingStartTime = (double)m_pTimer->Get();

	// Set flag that homing is not complete.
	isHomingComplete = false;

	// Set the current state to eHomingReverse.
	m_nCurrentState = eHomingReverse;
}

/******************************************************************************
	Description:	Stop - Stop the motor.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::Stop()
{
	// Stop the motor.
	m_pMotor->Set(ControlMode::PercentOutput, 0.000);

	// Set the current state to eIdle.
	m_nCurrentState = eIdle;
}

/******************************************************************************
	Description:	SetTolerance - Sets the tolerance of the PID in desired
					units of measure.
	Arguments:	 	dValue - Tolerance in the desired units.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetTolerance(double dValue)
{
	// Set the allowed error for the PID. This is in quadrature pulses.
	m_pMotor->ConfigAllowableClosedloopError(0, m_dPositionTolerance * m_dRevsPerUnit * m_nPulsesPerRev);

	// Set the member variable.
	m_dPositionTolerance = dValue;
}

/******************************************************************************
	Description:	GetTolerance - Returns the tolerance in the desired units.
	Arguments:	 	bUsePosition - Get the position tolerance.
	Returns: 		dValue - Tolerance in the desired units.
******************************************************************************/
double FalconMotion::GetTolerance(bool bUsePosition)
{
	// Create instance variables.
	double dTolerance = 0.000;

	if (bUsePosition)
	{
		dTolerance = m_dPositionTolerance;
	}
	else
	{
		dTolerance = m_dVelocityTolerance;
	}

	return dTolerance;
}

/******************************************************************************
	Description:	SetPositionSoftLimits - Sets soft limits for minimum and maximum travel.
	Arguments:	 	dMinValue - Minimum travel distance.
					dMaxValue - Maximum travel distance.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetPositionSoftLimits(double dMinValue, double dMaxValue)
{
	// Set the member variables.
	m_dLowerPositionSoftLimit	= dMinValue;
	m_dUpperPositionSoftLimit	= dMaxValue;
}

/******************************************************************************
	Description:	SetVelocitySoftLimits - Sets soft limits for minimum and maximum speed.
	Arguments:	 	dMinValue - Minimum travel distance.
					dMaxValue - Maximum travel distance.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetVelocitySoftLimits(double dMinValue, double dMaxValue)
{
	// Set the member variables.
	m_dLowerVelocitySoftLimit	= dMinValue;
	m_dUpperVelocitySoftLimit	= dMaxValue;
}

/******************************************************************************
	Description:	ConfigLimitSwitches - Sets up the limit switches as
					normally open or normally closed.
	Arguments:	 	bool bFwdLimit - True if normally open, false if normally closed.
					bool bRevLimit - True if normally open, false if normally closed.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::ConfigLimitSwitches(bool bFwdLimit, bool bRevLimit)
{
	// Set the member variables.
	isFwdLimitSwitchNormallyOpen = bFwdLimit;
	isRevLimitSwitchNormallyOpen = bRevLimit;

	m_pMotor->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
											(bFwdLimit ? LimitSwitchNormal::LimitSwitchNormal_NormallyOpen :
														 LimitSwitchNormal::LimitSwitchNormal_NormallyClosed));
	m_pMotor->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
											(bRevLimit ? LimitSwitchNormal::LimitSwitchNormal_NormallyOpen :
														 LimitSwitchNormal::LimitSwitchNormal_NormallyClosed));
}

/******************************************************************************
	Description:	SetAccumIZone - sets the IZone for the accumulated integral.
	Arguments:	 	double dIZone - The accumulated integral is reset to zero
					when the error exceeds this value. This value is in the
					units of measure.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetAccumIZone(double dIZone)
{
	// Set the member variable.
	m_dIZone = dIZone;

	// Set the Integral Zone. Accumulated integral is reset to zero when the error exceeds this value.
	m_pMotor->Config_IntegralZone(0, (m_dIZone * m_dRevsPerUnit * m_nPulsesPerRev));
}

/******************************************************************************
	Description:	IsFwdLimitSwitchPressed - Returns true if forward limit
					switch is pressed, false otherwise.
	Arguments:	 	None
	Returns: 		bool - True if pressed, false otherwise
******************************************************************************/
bool FalconMotion::IsFwdLimitSwitchPressed()
{
	return ((isFwdLimitSwitchNormallyOpen && m_pMotor->GetSensorCollection().IsFwdLimitSwitchClosed()) ||
		   (!isFwdLimitSwitchNormallyOpen && !m_pMotor->GetSensorCollection().IsFwdLimitSwitchClosed()));
}

/******************************************************************************
	Description:	IsRevLimitSwitchPressed - Returns true if reverse limit
					switch is pressed, false otherwise.
	Arguments:	 	None
	Returns: 		bool - True if pressed, false otherwise
******************************************************************************/
bool FalconMotion::IsRevLimitSwitchPressed()
{
	return ((isRevLimitSwitchNormallyOpen && m_pMotor->GetSensorCollection().IsRevLimitSwitchClosed()) ||
		   (!isRevLimitSwitchNormallyOpen && !m_pMotor->GetSensorCollection().IsRevLimitSwitchClosed()));
}

/******************************************************************************
	Description:	IsAtSetpoint - Returns whether or not the motor has reached
					the desired setpoint.
	Arguments:	 	None
	Returns: 		bool - True if at setpoint, false otherwise.
******************************************************************************/
bool FalconMotion::IsAtSetpoint()
{
	// Create instance variables.
	bool bIsAtSetpoint = false;

	if (isUsePosition)
	{
		bIsAtSetpoint = (((fabs(GetSetpoint() - GetActual(isUsePosition))) < m_dPositionTolerance) && (fabs(m_pMotor->GetMotorOutputVoltage()) < 1.000));
	}
	else
	{
		bIsAtSetpoint = (((fabs(GetSetpoint() - GetActual(isUsePosition))) < m_dVelocityTolerance) && (fabs(m_pMotor->GetMotorOutputVoltage()) < 1.000));
	}

	return bIsAtSetpoint;
}

/******************************************************************************
	Description:	ResetEncoderPosition - Sets the encoder position to zero.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::ResetEncoderPosition()
{
	// Reset the encoder count to zero.
	m_pMotor->SetSelectedSensorPosition(0);
}

/******************************************************************************
	Description:	SetPeakOutputPercent - Sets the maximum output for the
					motors. This is in PercentOutput (-1, to 1).
	Arguments:	 	double dMaxFwdOutput - The maximum forward output.
					double dMaxRevOutput - The maximum reverse output.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetPeakOutputPercent(double dMaxFwdOutput, double dMaxRevOutput)
{
	m_pMotor->ConfigPeakOutputForward(dMaxFwdOutput);
	m_pMotor->ConfigPeakOutputReverse(dMaxRevOutput);
}

/******************************************************************************
	Description:	SetNominalOutputVoltage - Sets the nominal output for the
					motors. This is in PercentOutput (-1, to 1).
	Arguments:	 	double dNominalFwdOutput - The nominal forward output.
					double dNominalRevOutput - The nominal reverse output.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetNominalOutputVoltage(double dNominalFwdOutput, double dNominalRevOutput)
{
	m_pMotor->ConfigNominalOutputForward(dNominalFwdOutput);
	m_pMotor->ConfigNominalOutputReverse(dNominalRevOutput);
}

/******************************************************************************
	Description:	SetOpenLoopRampRate - Sets the acceleration for open loop.
	Arguments:	 	double dOpenLoopRampRate - Acceleration in seconds from
					off to full output.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetOpenLoopRampRate(double dOpenLoopRampRate)
{
	m_pMotor->ConfigOpenloopRamp(dOpenLoopRampRate);
}

/******************************************************************************
	Description:	SetClosedLoopRampRate - Sets the acceleration for closed loop.
	Arguments:	 	double dClosedLoopRampRate - Acceleration in seconds from
					off to full output.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetClosedLoopRampRate(double dClosedLoopRampRate)
{
	m_pMotor->ConfigClosedloopRamp(dClosedLoopRampRate);
}

/******************************************************************************
	Description:	SetMotorNeutralMode - Sets the stop mode to brake or coast.
	Arguments:	 	int nMode - Mode, 1 is coast, 2 is brake.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetMotorNeutralMode(int nMode)
{
	m_pMotor->SetNeutralMode((nMode == 1) ? NeutralMode::Coast : NeutralMode::Brake);
}

/******************************************************************************
	Description:	GetCurrentPositionInUnits - Returns the current position in units.
	Arguments:	 	None
	Returns: 		double - Position of the motor.
******************************************************************************/
double FalconMotion::GetActual(bool bUsePosition)
{
	// Create instance variables.
	double dActual = 0.0;

	if (bUsePosition)
	{
		dActual = (m_pMotor->GetSelectedSensorPosition() / m_dRevsPerUnit / m_nPulsesPerRev);
	}
	else
	{
		dActual = (m_pMotor->GetSelectedSensorVelocity() / m_dRevsPerUnit / m_nPulsesPerRev) * m_dTimeUnitInterval;
	}

	return dActual;
}

/******************************************************************************
	Description:	SetHomeSpeeds - Sets the home speeds for the state machine.
	Arguments:	 	double dFwdSpeed - Speed for homing forward, coming off of
					home switch.
					double dRevSpeed - Speed for homing backward, moving towards
					home switch.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetHomeSpeeds(double dFwdSpeed, double dRevSpeed)
{
	m_dFwdHomeSpeed = dFwdSpeed;
	m_dRevHomeSpeed = dRevSpeed;
}

/******************************************************************************
	Description:	SetPulsesPerRev - Sets the pulses per revolution for the PID
					controller.
	Arguments:	 	int nPPR - Encoder pulses per revolution.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetPulsesPerRev(int nPPR)
{
	m_nPulsesPerRev = nPPR;
}

/******************************************************************************
	Description:	SetRevsPerUnit - Sets the revolutions per unit of measure.
	Arguments:	 	double dRPU - Revolutions per unit of measure.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetRevsPerUnit(double dRPU)
{
	m_dRevsPerUnit = dRPU;
}

/******************************************************************************
	Description:	SetPIDValues - Sets the PID and Feed Forward gain values.
	Arguments:	 	double dProportional 	- Proportion Gain
					double dIntegral		- Integral Gain
					double dDerivative		- Derivative Gain
					bool   bUsePosition		- Set Position PID Values
					double dFeedForward		- Feed Forward Gain
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetPIDValues(double dProportional, double dIntegral, double dDerivative, double dFeedForward)
{
	// Set PID values for either position or velocity.
	m_pMotor->Config_kP(0, dProportional);
	m_pMotor->Config_kI(0, dIntegral);
	m_pMotor->Config_kD(0, dDerivative);
	m_pMotor->Config_kF(0, dFeedForward);
}

/******************************************************************************
	Description:	SetMotorInverted - Inverts the motor output.
	Arguments:	 	bool bInverted - True to invert motor output.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetMotorInverted(bool bInverted)
{
	m_pMotor->SetInverted(bInverted);
}

/******************************************************************************
	Description:	SetSensorInverted - Inverts the sensor input.
	Arguments:	 	bool bInverted - True to invert sensor input.
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetSensorInverted(bool bInverted)
{
	m_pMotor->SetSensorPhase(bInverted);
}

/******************************************************************************
	Description:	ClearStickyFaults - Clears the controller's sticky faults.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::ClearStickyFaults()
{
	m_pMotor->ClearStickyFaults();
}

/******************************************************************************
	Description:	SetManualSpeed - Set the Manual Move Speed.
	Arguments:	 	double dForward, double dReverse
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetManualSpeed(double dForward, double dReverse)
{
	m_dFwdMoveSpeed = dForward;
	m_dRevMoveSpeed = dReverse;
}

/******************************************************************************
	Description:	SetAcceleration - Set the Motion Magic Acceleration.
	Arguments:	 	double dRPS
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetAcceleration(double dRPS)
{
	m_pMotor->ConfigMotionAcceleration(dRPS);
}

/******************************************************************************
	Description:	SetCruiseRPM - Set the Motion Magic Cruise RPM.
	Arguments:	 	double dRPM
	Returns: 		Nothing
******************************************************************************/
void FalconMotion::SetCruiseRPM(double dRPM)
{
	m_pMotor->ConfigMotionCruiseVelocity(dRPM);
}
///////////////////////////////////////////////////////////////////////////////