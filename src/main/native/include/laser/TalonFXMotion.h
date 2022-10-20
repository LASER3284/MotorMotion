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

#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include "laser/MotorMotion.h"

///////////////////////////////////////////////////////////////////////////////


namespace laser {

namespace talonfx {
    /*
    NOTE: this section of the header assumes default values that may not apply to
        your robot or mechanism; for instance, the defaultFalconMotionPulsesPerRev
        assumes a gearing of 50:9 (about 5.56). RevsPerUnit assumes 4 inch wheels.
        Many of the default values can be changed for your convenience and are only
        defaults.
    */

    // Default Constants set for drive motors.
    const int	 	defaultFalconMotionPulsesPerRev             =  	 11386;		// Encoder Pulses Per Revolution of wheel (Gearing * 2048).
    const double 	defaultFalconMotionRevsPerUnit              =    (1.000 / (4 * 3.1415));	// Revolutions per unit of measure. (1 revs(Encoder)/(4 in * PI))
    const double	defaultFalconMotionTimeUnitInterval         =	10.000;		// Falcon velocity returns rotations/100ms. (x10 for seconds)
    const double 	defaultFalconMotionFwdHomeSpeed             =    0.000;		// Homing forward speed (set to zero because drive motors don't home)
    const double 	defaultFalconMotionRevHomeSpeed             =    0.000;		// Homing reverse speed (set to zero because drive motors don't home)
    const double 	defaultFalconMotionPositionProportional     =    0.020; 	// Default proportional value for position.
    const double 	defaultFalconMotionPositionIntegral         =    0.000;		// Default integral value for position.
    const double 	defaultFalconMotionPositionDerivative       =    0.000;		// Default derivative value for position.
    const double	defaultFalconMotionVelocityProportional     =   0.0006;		// Default proportional value for velocity.
    const double 	defaultFalconMotionVelocityIntegral         =    0.000;		// Default integral value for velocity.
    const double	defaultFalconMotionVelocityDerivative       =    0.000;		// Default derivative value for velocity.
    const double	defaultFalconMotionFeedForward              =    0.350;		// Default feed forward value.
    const double 	defaultFalconMotionVoltageRampRate          =    0.250;		// Default voltage ramp rate. This is in seconds from neutral to full output.
    const double 	defaultFalconMotionPositionTolerance        =    0.250;		// Default tolerance for position in desired units.
    const double	defaultFalconMotionVelocityTolerance        = 	 1.000;		// Default tolerance for velocity in desired units.
    const double 	defaultFalconMotionLowerPositionSoftLimit   = -250.000; 	// Default lower position soft limit. This is in desired units.
    const double 	defaultFalconMotionUpperPositionSoftLimit   =  250.000; 	// Default upper position soft limit. This is in desired units.
    const double	defaultFalconMotionLowerVelocitySoftLimit   = -182.000;		// Default lower velocity soft limit. This is in desired units.
    const double	defaultFalconMotionUpperVelocitySoftLimit   =  182.000;		// Default upper velocity soft limit. This is in desired units.
    const double 	defaultFalconMotionIZone                    =    5.000;		// Default IZone value. This is in the desired units.
    const double 	defaultFalconMotionMaxHomingTime            =    0.000;		// Default Maximum allowable time to home. Zero to disable timeout. This is in seconds.
    const double 	defaultFalconMotionMaxFindingTime           =    0.000;		// Default Maximum allowable time to move to position. Zero to disable timeout. This is in seconds.
    const double	defualtFalconMotionManualFwdSpeed           =	 0.500;
    const double	defualtFalconMotionManualRevSpeed           =	-0.500;

    ////////////////////////////////////////////////////////////////////////////

    class TalonFXMotion : public MotorMotion<ctre::phoenix::ErrorCode, ctre::phoenix::motorcontrol::can::WPI_TalonFX> {
        public:
            TalonFXMotion(int nDeviceID);
            ~TalonFXMotion();

            // Prototype methods - overrides of virtual methods
            void	ClearStickyFaults() override;
            void	ConfigLimitSwitches(bool bFwdLimitNormallyOpen, bool bRevLimitNormallyOpen) override;
            double	GetActual(bool bUsePosition);
            double	GetSetpoint();
            double  GetTolerance(bool bUsePosition);
            bool    IsAtSetpoint();
            bool	IsFwdLimitSwitchPressed();
            bool	IsRevLimitSwitchPressed();
            void	ResetEncoderPosition();
            void	SetAcceleration(double dRPS);
            void	SetAccumIZone(double dIZone);
            void	SetClosedLoopRampRate(double dClosedLoopRampRate);
            void	SetCruiseRPM(double dRPM);
            void	SetHomeSpeeds(double dFwdSpeed, double dRevSpeed);
            void	SetManualSpeed(double dForward, double dReverse);
            void	SetMotorInverted(bool bInverted);
            void	SetMotorNeutralMode(int nMode);
            void	SetNominalOutputVoltage(double dNominalFwdOutput, double dNominalRevOutput);
            void	SetOpenLoopRampRate(double dOpenLoopRampRate);
            void	SetPeakOutputPercent(double dMaxFwdOutput, double dMaxRevOutput);
            void	SetPIDValues(double dProportional, double dIntegral, double dDerivative, double dFeedForward = 0.000);
            void	SetPulsesPerRev(int nPPR);
            void	SetRevsPerUnit(double dRPU);
            void	SetSensorInverted(bool bInverted);
            void	SetSetpoint(double dSetpoint, bool bUsePosition);
            void	SetMotorVoltage(double dVoltage);
            void	SetPositionSoftLimits(double dMinValue, double dMaxValue);
            void	SetVelocitySoftLimits(double dMinValue, double dMaxValue);
            void	SetTolerance(double dValue);
            void	StartHoming();
            void	Stop() override;
            void	Tick();

            // One-liners - overrides of virtual methods
            bool	IsHomingComplete()							{ return isHomingComplete;												};
            void	SetMaxHomingTime(double dMaxHomingTime)		{ m_dMaxHomingTime = dMaxHomingTime;									};
            void	SetMaxFindingTime(double dMaxFindingTime)	{ m_dMaxFindingTime = dMaxFindingTime;									};
            State	GetState()									{ return m_nCurrentState;												};
            void	SetState(State nNewState)					{ m_nCurrentState = nNewState;											};
            double	GetMotorCurrent()							{ return m_pMotor->GetOutputCurrent();									};
            double	GetMotorVoltage()							{ return m_pMotor->GetMotorOutputVoltage(); 							};
            double	GetRevsPerUnit()							{ return m_dRevsPerUnit;												};
            int		GetPulsesPerRev()							{ return m_nPulsesPerRev;												};
            int		GetRawEncoderCounts()						{ return m_pMotor->GetSelectedSensorPosition();	                        };
            void	BackOffHome(bool bBackOff)					{ isBackOffHome = bBackOff;											};
            void	UseMotionMagic(bool bEnabled)				{ isMotionMagic = bEnabled;											};

        private:
            // members
            bool                    isFwdLimitSwitchNormallyOpen;
            bool                    isRevLimitSwitchNormallyOpen;
            bool                    isHomingComplete;
            bool                    isBackOffHome;
            bool                    isMotionMagic;
            bool                    isUsePosition;
            int	                    pulsesPerRev;
            int	                    deviceID;
            double                  setpoint;
            double                  revsPerUnit;
            double                  timeUnitInterval;
            double                  fwdMoveSpeed;
            double                  revMoveSpeed;
            double                  fwdHomeSpeed;
            double                  revHomeSpeed;
            double                  positionProportional;
            double                  positionIntegral;
            double                  positionDerivative;
            double                  velocityProportional;
            double                  velocityIntegral;
            double                  velocityDerivative;
            double                  positionTolerance;
            double                  velocityTolerance;
            double                  lowerPositionSoftLimit;
            double                  upperPositionSoftLimit;
            double                  lowerVelocitySoftLimit;
            double                  upperVelocitySoftLimit;
            double                  izone;
            double                  maxHomingTime;
            double                  maxFindingTime;
            double                  homingStartTime;
            double                  findingStartTime;
    }; // class TalonFXMotion

} // namespace talonfx

} // namespace laser
