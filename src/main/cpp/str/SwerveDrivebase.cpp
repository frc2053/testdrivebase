// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/SwerveDrivebase.h"

#include <fmt/format.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/Utils.hpp>

SwerveDrivebase::SwerveDrivebase(
  std::function<void(SwerveDriveState)> telemFunc)
  : telemetryFunction(telemFunc)
{
  frc::SmartDashboard::PutData("FL Module Tuner", &modules[0]);
  frc::SmartDashboard::PutData("FR Module Tuner", &modules[1]);
  frc::SmartDashboard::PutData("BL Module Tuner", &modules[2]);
  frc::SmartDashboard::PutData("BR Module Tuner", &modules[3]);
}

void SwerveDrivebase::SetupOdomStuff()
{
  for (int i = 0; i < 4; i++) {
    std::array<ctre::phoenix6::BaseStatusSignal*, 4> signals
      = modules[i].GetSignals();
    allSignals[(i * 4) + 0] = signals[0];
    allSignals[(i * 4) + 1] = signals[1];
    allSignals[(i * 4) + 2] = signals[2];
    allSignals[(i * 4) + 3] = signals[3];
  }
  allSignals[allSignals.size() - 2] = &imu.GetYaw();
  allSignals[allSignals.size() - 1] = &imu.GetAngularVelocityZ();

  for (ctre::phoenix6::BaseStatusSignal* sig : allSignals) {
    sig->SetUpdateFrequency(250_Hz);
  }
}

void SwerveDrivebase::UpdateOdometry()
{
  ctre::phoenix::StatusCode status;
  status
    = ctre::phoenix6::BaseStatusSignal::WaitForAll(2.0 / 250_Hz, allSignals);

  lastTime = currentTime;
  currentTime = units::second_t{ctre::phoenix6::GetCurrentTimeSeconds()};
  averageLoopTime
    = lowpass.Calculate(peakRemover.Calculate(currentTime - lastTime));

  if (status.IsOK()) {
    successfulDaqs++;
  } else {
    failedDaqs++;
  }

  for (int i = 0; i < 4; i++) {
    modulePostions[i] = modules[i].GetPosition(false);
  }

  units::radian_t imuYaw
    = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
      imu.GetYaw(), imu.GetAngularVelocityZ());
  odometry.Update(frc::Rotation2d{imuYaw}, modulePostions);

  requestParameters.currentPose = odometry.GetEstimatedPosition().RelativeTo(
    frc::Pose2d{0_m, 0_m, fieldRelativeOffset});
  requestParameters.kinematics = kinematics;
  requestParameters.swervePositions = moduleLocations;
  requestParameters.timestamp = currentTime;

  requestToApply->Apply(requestParameters, modules);

  cachedState.failedDaqs = failedDaqs;
  cachedState.successfulDaqs = successfulDaqs;
  cachedState.moduleStates
    = {modules[0].GetCurrentState(), modules[1].GetCurrentState(),
      modules[2].GetCurrentState(), modules[3].GetCurrentState()};
  cachedState.pose = odometry.GetEstimatedPosition();
  cachedState.odometryPeriod = averageLoopTime;

  telemetryFunction(cachedState);

  if (successfulDaqs > 2) {
    validOdom = true;
  }
}

void SwerveDrivebase::SetControl(
  std::unique_ptr<RequestTypes::SwerveRequest> request)
{
  requestToApply = std::move(request);
}

void SwerveDrivebase::TareEverything()
{
  for (int i = 0; i < 4; i++) {
    modules[i].ResetPosition();
    modulePostions[i] = modules[i].GetPosition(true);
  }
  odometry.ResetPosition(imu.GetRotation2d(), modulePostions, frc::Pose2d{});
}

void SwerveDrivebase::SeedFieldRelative()
{
  fieldRelativeOffset = GetState().pose.Rotation();
}

void SwerveDrivebase::SeedFieldRelative(frc::Pose2d location)
{
  fieldRelativeOffset = location.Rotation();
  odometry.ResetPosition(location.Rotation(), modulePostions, location);
}

SwerveDriveState SwerveDrivebase::GetState() { return cachedState; }

void SwerveDrivebase::AddVisionMeasurement(frc::Pose2d visionRobotPose,
  units::second_t timestamp, wpi::array<double, 3> visionMeasurementStdDevs)
{
  odometry.AddVisionMeasurement(
    visionRobotPose, timestamp, visionMeasurementStdDevs);
}

void SwerveDrivebase::AddVisionMeasurement(
  frc::Pose2d visionRobotPose, units::second_t timestamp)
{
  odometry.AddVisionMeasurement(visionRobotPose, timestamp);
}

void SwerveDrivebase::SetVisionMeasurementStdDevs(
  wpi::array<double, 3> visionMeasurementStdDevs)
{
  odometry.SetVisionMeasurementStdDevs(visionMeasurementStdDevs);
}

void SwerveDrivebase::UpdateSimState(
  units::second_t dt, units::volt_t supplyVoltage)
{
  simDrivetrain.Update(dt, supplyVoltage, modules);
}

bool SwerveDrivebase::IsOdometryValid() { return validOdom; }
