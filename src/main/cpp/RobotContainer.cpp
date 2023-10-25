// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

RobotContainer::RobotContainer() { ConfigureBindings(); }

void RobotContainer::ConfigureBindings()
{
  drivebaseSub.SetDefaultCommand(drivebaseSub.ApplyRequest([this] {
    return std::make_unique<RequestTypes::FieldCentric>(
      drive
        .withVelocityX(-driverController.GetLeftY()
          * constants::drivebase::physical::MAX_DRIVE_SPEED)
        .withVelocityY(-driverController.GetLeftX()
          * constants::drivebase::physical::MAX_DRIVE_SPEED)
        .withRotationalRate(-driverController.GetRightX() * 3.14_rad_per_s));
  }));

  driverController.A().WhileTrue(drivebaseSub.ApplyRequest([this] {
    return std::make_unique<RequestTypes::FieldCentricFacingAngle>(
      driveAtAngle
        .withVelocityX(-driverController.GetLeftY()
          * constants::drivebase::physical::MAX_DRIVE_SPEED)
        .withVelocityY(-driverController.GetLeftX()
          * constants::drivebase::physical::MAX_DRIVE_SPEED)
        .withTargetDirection(180_deg));
  }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  return pathplanner::PathPlannerAuto("TestAuto").ToPtr();
}
