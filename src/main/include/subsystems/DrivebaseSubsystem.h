// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>

#include <functional>
#include <memory>

#include "str/SwerveDrivebase.h"
#include "str/SwerveTelemetry.h"

class DrivebaseSubsystem : public frc2::SubsystemBase, public SwerveDrivebase {
public:
  DrivebaseSubsystem();

  frc2::CommandPtr ApplyRequest(
    std::function<std::unique_ptr<RequestTypes::SwerveRequest>()>
      requestSupplier);

  void Periodic() override;
  void SimulationPeriodic() override;
  void SeedFieldRelative(frc::Pose2d location) override;

private:
  SwerveTelemetry telem{constants::drivebase::physical::MAX_DRIVE_SPEED};
};