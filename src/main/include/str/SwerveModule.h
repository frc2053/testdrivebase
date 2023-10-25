// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

class SwerveModule {
public:
  SwerveModule(int driveMotorId, int steerMotorId, int steerEncId,
    double steerEncOffset, bool invertDrive, bool invertSteer);
  void GoToState(frc::SwerveModuleState state, bool openLoop);

  frc::SwerveModulePosition GetPosition(bool refresh);
  frc::SwerveModulePosition GetCachedPosition();
  frc::SwerveModuleState GetCurrentState();
  void ResetPosition();
  std::array<ctre::phoenix6::BaseStatusSignal*, 4> GetSignals();

  void SetSteerMotorVolts(units::volt_t voltage);

  ctre::phoenix6::hardware::TalonFX driveMotor;
  ctre::phoenix6::hardware::TalonFX steerMotor;
  ctre::phoenix6::hardware::CANcoder steerEnc;

private:
  void ConfigureDriveMotor(bool invertDrive);
  void ConfigureSteerEncoder(double encOffset);
  void ConfigureSteerMotor(bool invertSteer);

  frc::SwerveModulePosition internalState;

  ctre::phoenix6::StatusSignal<units::turn_t> steerAngleSig
    = steerMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> steerAngleVelSig
    = steerMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::turn_t> drivePositionSig
    = driveMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> driveVelocitySig
    = driveMotor.GetVelocity();

  ctre::phoenix6::controls::MotionMagicVoltage angleSetter{0_rad};
  ctre::phoenix6::controls::VelocityTorqueCurrentFOC velocityTorqueSetter{
    0_rad_per_s};
  ctre::phoenix6::controls::VoltageOut voltageOpenLoopSetter{0_V};
  ctre::phoenix6::controls::VoltageOut identifySteerSetter{0_V};
};
