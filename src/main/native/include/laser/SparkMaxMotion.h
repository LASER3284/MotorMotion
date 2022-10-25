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

#include <rev/CANSparkMax.h>
////////////////////////////////////////////////////////////////////////////////

namespace laser {

namespace sparkmax {

    namespace defaults {

    }

    class SparkMotion {
        public:
            // Method Prototypes.
            SparkMotion(int nDeviceID);
            ~SparkMotion();

            void	ClearStickyFaults();
            void	ConfigLimitSwitches(bool bFwdLimitNormallyOpen, bool bRevLimitNormallyOpen);
            double	GetActual();
            double	GetSetpoint();
            double  GetTolerance();
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
            void	SetSetpoint(double dSetpoint, bool bUsePosition);
            void	SetPositionSoftLimits(double dMinValue, double dMaxValue);
            void	SetVelocitySoftLimits(double dMinValue, double dMaxValue);
            void	SetTolerance(double dValue);
            void	StartHoming();
            void	Stop();
            void	Tick();

            // One-line Methods.
            CANSparkMax*	GetMotorPointer()			    	{ return m_pMotor;														};
            bool	IsReady()									{ return m_bReady;														};
            bool	IsHomingComplete()							{ return m_bHomingComplete;												};
            void	SetMaxHomingTime(double dMaxHomingTime)		{ m_dMaxHomingTime = dMaxHomingTime;									};
            void	SetMaxFindingTime(double dMaxFindingTime)	{ m_dMaxFindingTime = dMaxFindingTime;									};
            State	GetState()									{ return m_nCurrentState;												};
            void	SetState(State nNewState)					{ m_nCurrentState = nNewState;											};
            double	GetMotorCurrent()							{ return m_pMotor->GetOutputCurrent();									};
            double	GetMotorVoltage()							{ return m_pMotor->GetBusVoltage(); 				        			};
            double	GetRevsPerUnit()							{ return m_dRevsPerUnit;												};
            int		GetPulsesPerRev()							{ return m_nPulsesPerRev;												};
        //	int		GetRawEncoderCounts()						{ return m_pMotor->GetSelectedSensorPosition();	                        };
            void	BackOffHome(bool bBackOff)					{ m_bBackOffHome = bBackOff;											};
            void	UseMotionMagic(bool bEnabled)				{ m_bMotionMagic = bEnabled;											};

        private:
            // Object Pointers.
            CANSparkMax*            m_pMotor;
            SparkMaxPIDController*  m_pPIDController;
            Timer*                  m_pTimer;

            // Member Variables.
            bool					m_bFwdLimitSwitchNormallyOpen;
            bool					m_bRevLimitSwitchNormallyOpen;
            bool					m_bHomingComplete;
            bool					m_bReady;
            bool					m_bBackOffHome;
            bool					m_bMotionMagic;
            bool					m_bUsePosition;
            int						m_nPulsesPerRev;
            int						m_nDeviceID;
            double					m_dSetpoint;
            double					m_dTimeUnitInterval; //TODO: Still unknown. Needs testing.
            double					m_dRevsPerUnit;
            double					m_dFwdMoveSpeed;
            double					m_dRevMoveSpeed;
            double					m_dFwdHomeSpeed;
            double					m_dRevHomeSpeed;
            double					m_dTolerance;
            double					m_dLowerPositionSoftLimit;
            double					m_dUpperPositionSoftLimit;
            double					m_dLowerVelocitySoftLimit;
            double					m_dUpperVelocitySoftLimit;
            double					m_dIZone;
            double					m_dMaxHomingTime;
            double					m_dMaxFindingTime;
            double					m_dHomingStartTime;
            double					m_dFindingStartTime;
            State					m_nCurrentState;
    };

} // namespace sparkmax

} // namespace laser