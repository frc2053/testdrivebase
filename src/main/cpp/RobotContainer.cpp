// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

RobotContainer::RobotContainer() { ConfigureBindings(); }

void RobotContainer::ConfigureBindings()
{
  frc::SmartDashboard::PutData("Characterize Modules", charModulesCmd.get());

  drivebaseSub.SetDefaultCommand(drivebaseSub.ApplyRequest([this] {
    return std::make_unique<RequestTypes::FieldCentric>(
      drive
        .withVelocityX(-driverController.GetLeftY()
          * constants::drivebase::physical::MAX_DRIVE_SPEED)
        .withDeadband(constants::drivebase::physical::MAX_DRIVE_SPEED * .2)
        .withVelocityY(-driverController.GetLeftX()
          * constants::drivebase::physical::MAX_DRIVE_SPEED)
        .withRotationalRate(-driverController.GetRightX()
          * constants::drivebase::physical::MAX_ROTATION_SPEED)
        .withRotationalDeadband(
          constants::drivebase::physical::MAX_ROTATION_SPEED * .1));
  }));

  driverController.A().WhileTrue(drivebaseSub.ApplyRequest([this] {
    return std::make_unique<RequestTypes::FieldCentricFacingAngle>(
      driveAtAngle
        .withDeadband(constants::drivebase::physical::MAX_DRIVE_SPEED * .2)
        .withVelocityX(-driverController.GetLeftY()
          * constants::drivebase::physical::MAX_DRIVE_SPEED)
        .withVelocityY(-driverController.GetLeftX()
          * constants::drivebase::physical::MAX_DRIVE_SPEED)
        .withTargetDirection(180_deg));
  }));

  driverController.X().WhileTrue(drivebaseSub.ApplyRequest([this] {
    return std::make_unique<RequestTypes::FieldCentricFacingAngle>(
      driveAtAngle
        .withDeadband(constants::drivebase::physical::MAX_DRIVE_SPEED * .2)
        .withVelocityX(-driverController.GetLeftY()
          * constants::drivebase::physical::MAX_DRIVE_SPEED)
        .withVelocityY(-driverController.GetLeftX()
          * constants::drivebase::physical::MAX_DRIVE_SPEED)
        .withTargetDirection(90_deg));
  }));

  driverController.B().WhileTrue(drivebaseSub.ApplyRequest([this] {
    return std::make_unique<RequestTypes::FieldCentricFacingAngle>(
      driveAtAngle
        .withDeadband(constants::drivebase::physical::MAX_DRIVE_SPEED * .2)
        .withVelocityX(-driverController.GetLeftY()
          * constants::drivebase::physical::MAX_DRIVE_SPEED)
        .withVelocityY(-driverController.GetLeftX()
          * constants::drivebase::physical::MAX_DRIVE_SPEED)
        .withTargetDirection(-90_deg));
  }));

  driverController.Y().WhileTrue(drivebaseSub.ApplyRequest([this] {
    return std::make_unique<RequestTypes::FieldCentricFacingAngle>(
      driveAtAngle
        .withDeadband(constants::drivebase::physical::MAX_DRIVE_SPEED * .2)
        .withVelocityX(-driverController.GetLeftY()
          * constants::drivebase::physical::MAX_DRIVE_SPEED)
        .withVelocityY(-driverController.GetLeftX()
          * constants::drivebase::physical::MAX_DRIVE_SPEED)
        .withTargetDirection(0_deg));
  }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  return pathplanner::PathPlannerAuto("TestAuto").ToPtr();
}
