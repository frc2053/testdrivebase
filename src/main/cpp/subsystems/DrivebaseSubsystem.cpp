// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/DrivebaseSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/RunCommand.h>

DrivebaseSubsystem::DrivebaseSubsystem()
  : SwerveDrivebase(
    [this](SwerveDriveState state) { return telem.Telemeterize(state); })
{
  SetupAutoBuilder();
}

frc2::CommandPtr DrivebaseSubsystem::ApplyRequest(
  std::function<std::unique_ptr<RequestTypes::SwerveRequest>()> requestSupplier)
{
  return frc2::RunCommand{
    [this, requestSupplier] { SetControl(requestSupplier()); }, {this}}
    .ToPtr();
}

// This method will be called once per scheduler run
void DrivebaseSubsystem::Periodic() { }

void DrivebaseSubsystem::SimulationPeriodic()
{
  UpdateSimState(constants::ROBOT_DT, 12_V);
}

void DrivebaseSubsystem::SeedFieldRelative(frc::Pose2d location)
{
  std::unique_lock<std::shared_mutex> writeLock(lock);
  odometry.ResetPosition(
    frc::Rotation2d{imu.GetYaw().GetValue()}, modulePostions, location);
}

void DrivebaseSubsystem::SetupAutoBuilder()
{
  pathplanner::AutoBuilder::configureHolonomic(
    [this] { return GetState().pose; },
    [this](frc::Pose2d resetPose) { SeedFieldRelative(resetPose); },
    [] { return frc::ChassisSpeeds{}; },
    [this](frc::ChassisSpeeds speeds) {
      SetControl(std::make_unique<RequestTypes::ApplyChassisSpeeds>(
        autoRequest.withSpeeds(speeds)));
    },
    pathplanner::HolonomicPathFollowerConfig(
      pathplanner::PIDConstants{constants::drivebase::gains::TRANSLATION_P,
        constants::drivebase::gains::TRANSLATION_I,
        constants::drivebase::gains::TRANSLATION_D},
      pathplanner::PIDConstants{constants::drivebase::gains::TRANSLATION_P,
        constants::drivebase::gains::TRANSLATION_I,
        constants::drivebase::gains::TRANSLATION_D},
      constants::drivebase::physical::MAX_DRIVE_SPEED,
      constants::drivebase::physical::WHEELBASE_LENGTH / 2,
      pathplanner::ReplanningConfig{false}),
    this);
}
