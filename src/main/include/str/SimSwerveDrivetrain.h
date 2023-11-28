// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/simulation/DCMotorSim.h>
#include <str/SwerveModule.h>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>

#include "Constants.h"

class SimSwerveDrivetrain {
public:
  SimSwerveDrivetrain(frc::SwerveDriveKinematics<4> kine,
    ctre::phoenix6::hardware::Pigeon2& pigeon);
  void Update(units::second_t deltaTime, units::volt_t supplyVoltage,
    std::array<SwerveModule, 4>& modules);

  static Eigen::Matrix<double, 2, 1> CalculateX(
    const Eigen::Matrix<double, 2, 2>& discA,
    const Eigen::Matrix<double, 2, 1>& discB,
    const Eigen::Matrix<double, 2, 1>& x, units::volt_t input,
    units::volt_t kS);
  void Reset(const frc::Pose2d& pose, bool preserveMotion);
  void Reset(const frc::Pose2d& pose,
    const std::array<Eigen::Matrix<double, 2, 1>, 4>& moduleDriveStates,
    const std::array<Eigen::Matrix<double, 2, 1>, 4>& moduleSteerStates);
  frc::Pose2d GetPose() const;
  std::array<frc::SwerveModulePosition, 4> GetModulePositions() const;
  std::array<frc::SwerveModulePosition, 4> GetNoisyModulePositions(
    units::meter_t driveStdDev, units::radian_t steerStdDev);
  std::array<frc::SwerveModuleState, 4> GetModuleStates();
  std::array<Eigen::Matrix<double, 2, 1>, 4> GetDriveStates() const;
  std::array<Eigen::Matrix<double, 2, 1>, 4> GetSteerStates() const;
  units::radians_per_second_t GetOmega() const;
  units::ampere_t GetCurrentDraw(const frc::DCMotor& motor,
    units::radians_per_second_t velocity, units::volt_t inputVolts,
    units::volt_t batteryVolts) const;
  std::array<units::ampere_t, 4> GetDriveCurrentDraw() const;
  std::array<units::ampere_t, 4> GetSteerCurrentDraw() const;
  units::ampere_t GetTotalCurrentDraw() const;

private:
  frc::SwerveDriveKinematics<4> kinematics;
  ctre::phoenix6::sim::Pigeon2SimState pigeonSim;

  std::random_device rd{};
  std::mt19937 generator{rd()};
  std::normal_distribution<double> randDist{0.0, 1.0};
  const frc::LinearSystem<2, 1, 2> drivePlant;
  const frc::DCMotor driveMotor{frc::DCMotor::Falcon500(1)};
  const frc::LinearSystem<2, 1, 2> steerPlant;
  const frc::DCMotor steerMotor{frc::DCMotor::Falcon500(1)};
  std::array<units::volt_t, 4> driveInputs{};
  std::array<Eigen::Matrix<double, 2, 1>, 4> driveStates{};
  std::array<units::volt_t, 4> steerInputs{};
  std::array<Eigen::Matrix<double, 2, 1>, 4> steerStates{};
  frc::Pose2d pose{frc::Pose2d{}};
  units::radians_per_second_t omega{0};
  frc::Rotation2d lastAngle{};
};
