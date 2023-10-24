// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/SwerveTelemetry.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>

SwerveTelemetry::SwerveTelemetry(units::meters_per_second_t maxSpeed)
  : maximumSpeed(maxSpeed)
{
}

void SwerveTelemetry::Telemeterize(SwerveDriveState state)
{
  frc::Pose2d pose = state.pose;
  fieldTypePub.Set("Field2d");
  double fieldArr[]
    = {pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value()};
  fieldPub.Set(fieldArr);

  units::second_t currentTime
    = units::second_t{ctre::phoenix6::GetCurrentTimeSeconds()};
  units::second_t diffTime = currentTime - lastTime;
  lastTime = currentTime;

  frc::Translation2d distanceDiff = (pose - lastPose).Translation();
  lastPose = pose;

  frc::Translation2d velocities = distanceDiff / diffTime.value();

  speed.Set(velocities.Norm().value());
  velocityX.Set(velocities.X().value());
  velocityY.Set(velocities.Y().value());
  odomPeriod.Set(1.0 / state.odometryPeriod.value());

  for (size_t i = 0; i < 4; i++) {
    moduleSpeedVis[i]->SetAngle(state.moduleStates[i].angle.Degrees());
    moduleDirections[i]->SetLength(
      state.moduleStates[i].speed / (2 * maximumSpeed));
    frc::SmartDashboard::PutData("Module " + std::to_string(i), &moduleVis[i]);
  }

  double logValues[]
    = {pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value()};
  frc::DataLogManager::GetLog().AppendDoubleArray(
    logEntry, logValues, frc::Timer::GetFPGATimestamp().value());
  frc::DataLogManager::GetLog().AppendDouble(odomEntry,
    state.odometryPeriod.value(), frc::Timer::GetFPGATimestamp().value());
}
