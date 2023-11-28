// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/SimSwerveDrivetrain.h"

#include <wpi/array.h>

#include <iostream>

#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

SimSwerveDrivetrain::SimSwerveDrivetrain(
  frc::SwerveDriveKinematics<4> kine, ctre::phoenix6::hardware::Pigeon2& pigeon)
  : kinematics(kine)
  , pigeonSim(pigeon)
  , drivePlant{(Eigen::MatrixXd(2, 2) << 0.0, 1.0, 0.0,
                 -constants::drivebase::gains::DRIVE_KV
                   / constants::drivebase::gains::DRIVE_KA)
                 .finished(),
      Eigen::Matrix<double, 2, 1>{
        0.0, 1.0 / constants::drivebase::gains::DRIVE_KA},
      (Eigen::MatrixXd(2, 2) << 1.0, 0.0, 0.0, 1.0).finished(),
      Eigen::Matrix<double, 2, 1>{0.0, 0.0}}
  , steerPlant{(Eigen::MatrixXd(2, 2) << 0.0, 1.0, 0.0,
                 -constants::drivebase::gains::STEER_KV
                   / constants::drivebase::gains::STEER_KA)
                 .finished(),
      Eigen::Matrix<double, 2, 1>{
        0.0, 1.0 / constants::drivebase::gains::STEER_KA},
      (Eigen::MatrixXd(2, 2) << 1.0, 0.0, 0.0, 1.0).finished(),
      Eigen::Matrix<double, 2, 1>{0.0, 0.0}}
{
}

Eigen::Matrix<double, 2, 1> SimSwerveDrivetrain::CalculateX(
  const Eigen::Matrix<double, 2, 2>& discA,
  const Eigen::Matrix<double, 2, 1>& discB,
  const Eigen::Matrix<double, 2, 1>& x, units::volt_t input, units::volt_t kS)
{
  auto Ax = discA * x;
  double nextStateVel = Ax(1, 0);
  double inputToStop = nextStateVel / -discB(1, 0);
  double ksSystemEffect
    = std::clamp(inputToStop, -kS.to<double>(), kS.to<double>());

  nextStateVel += discB(1, 0) * ksSystemEffect;
  inputToStop = nextStateVel / -discB(1, 0);
  double signToStop = sgn(inputToStop);
  double inputSign = sgn(input.to<double>());
  double ksInputEffect = 0;

  if (std::abs(ksSystemEffect) < kS.to<double>()) {
    double absInput = std::abs(input.to<double>());
    ksInputEffect
      = -std::clamp(kS.to<double>() * inputSign, -absInput, absInput);
  } else if ((input.to<double>() * signToStop) > (inputToStop * signToStop)) {
    double absInput = std::abs(input.to<double>() - inputToStop);
    ksInputEffect
      = -std::clamp(kS.to<double>() * inputSign, -absInput, absInput);
  }

  auto sF = Eigen::Matrix<double, 1, 1>{
    input.to<double>() + ksSystemEffect + ksInputEffect};
  auto Bu = discB * sF;
  auto retVal = Ax + Bu;
  return retVal;
}

void SimSwerveDrivetrain::Reset(const frc::Pose2d& pose, bool preserveMotion)
{
  this->pose = pose;
  if (!preserveMotion) {
    for (int i = 0; i < 4; i++) {
      driveStates[i] = Eigen::Matrix<double, 2, 1>{0, 0};
      steerStates[i] = Eigen::Matrix<double, 2, 1>{0, 0};
    }
    omega = 0_rad_per_s;
  }
}

void SimSwerveDrivetrain::Reset(const frc::Pose2d& pose,
  const std::array<Eigen::Matrix<double, 2, 1>, 4>& moduleDriveStates,
  const std::array<Eigen::Matrix<double, 2, 1>, 4>& moduleSteerStates)
{
  this->pose = pose;
  driveStates = moduleDriveStates;
  steerStates = moduleSteerStates;
  omega = kinematics.ToChassisSpeeds(GetModuleStates()).omega;
}

frc::Pose2d SimSwerveDrivetrain::GetPose() const { return pose; }

std::array<frc::SwerveModulePosition, 4>
SimSwerveDrivetrain::GetModulePositions() const
{
  std::array<frc::SwerveModulePosition, 4> positions;
  for (int i = 0; i < 4; i++) {
    positions[i]
      = frc::SwerveModulePosition{units::meter_t{driveStates[i](0, 0)},
        frc::Rotation2d{units::radian_t{steerStates[i](0, 0)}}};
  }
  return positions;
}

std::array<frc::SwerveModulePosition, 4>
SimSwerveDrivetrain::GetNoisyModulePositions(
  units::meter_t driveStdDev, units::radian_t steerStdDev)
{
  std::array<frc::SwerveModulePosition, 4> positions;
  for (int i = 0; i < 4; i++) {
    positions[i] = frc::SwerveModulePosition{
      units::meter_t{driveStates[i](0, 0)} + randDist(generator) * driveStdDev,
      frc::Rotation2d{units::radian_t{steerStates[i](0, 0)}
        + randDist(generator) * steerStdDev}};
  }
  return positions;
}

std::array<frc::SwerveModuleState, 4> SimSwerveDrivetrain::GetModuleStates()
{
  std::array<frc::SwerveModuleState, 4> states;
  for (int i = 0; i < 4; i++) {
    states[i]
      = frc::SwerveModuleState{units::meters_per_second_t{driveStates[i](1, 0)},
        frc::Rotation2d{units::radian_t{steerStates[i](0, 0)}}};
  }
  return states;
}

std::array<Eigen::Matrix<double, 2, 1>, 4>
SimSwerveDrivetrain::GetDriveStates() const
{
  return driveStates;
}

std::array<Eigen::Matrix<double, 2, 1>, 4>
SimSwerveDrivetrain::GetSteerStates() const
{
  return steerStates;
}

units::radians_per_second_t SimSwerveDrivetrain::GetOmega() const
{
  return omega;
}

units::ampere_t SimSwerveDrivetrain::GetCurrentDraw(const frc::DCMotor& motor,
  units::radians_per_second_t velocity, units::volt_t inputVolts,
  units::volt_t batteryVolts) const
{
  units::volt_t effVolts = inputVolts - velocity / motor.Kv;
  if (inputVolts >= 0_V) {
    effVolts = std::clamp(effVolts, 0_V, inputVolts);
  } else {
    effVolts = std::clamp(effVolts, inputVolts, 0_V);
  }
  auto retVal = (inputVolts / batteryVolts) * (effVolts / motor.R);
  return retVal;
}

std::array<units::ampere_t, 4> SimSwerveDrivetrain::GetDriveCurrentDraw() const
{
  std::array<units::ampere_t, 4> currents;
  for (int i = 0; i < 4; i++) {
    units::radians_per_second_t speed
      = units::radians_per_second_t{driveStates[i](1, 0)}
      * constants::drivebase::physical::DRIVE_GEARING
      / (constants::drivebase::physical::WHEEL_DIAM / 2).to<double>();
    currents[i] = GetCurrentDraw(driveMotor, speed, driveInputs[i],
      frc::RobotController::GetBatteryVoltage());
  }
  return currents;
}

std::array<units::ampere_t, 4> SimSwerveDrivetrain::GetSteerCurrentDraw() const
{
  std::array<units::ampere_t, 4> currents;
  for (int i = 0; i < 4; i++) {
    units::radians_per_second_t speed = units::radians_per_second_t{
      steerStates[i](1, 0) * constants::drivebase::physical::STEER_GEARING};
    // TODO: If uncommented we get huge current values.. Not sure how to fix
    // atm. :(
    currents[i] = 20_A;
    // currents[i] = GetCurrentDraw(steerMotor, speed, steerInputs[i],
    // frc::RobotController::GetBatteryVoltage());
  }
  return currents;
}

units::ampere_t SimSwerveDrivetrain::GetTotalCurrentDraw() const
{
  units::ampere_t total{0};
  for (const auto& val : GetDriveCurrentDraw()) {
    total += val;
  }
  for (const auto& val : GetSteerCurrentDraw()) {
    total += val;
  }
  return total;
}

void SimSwerveDrivetrain::Update(units::second_t deltaTime,
  units::volt_t supplyVoltage, std::array<SwerveModule, 4>& modulesToApply)
{
  for (int i = 0; i < 4; i++) {
    auto [driveVoltSig, steerVoltSig] = modulesToApply[i].GetVoltageSignals();
    ctre::phoenix6::BaseStatusSignal::WaitForAll(
      0_s, *driveVoltSig, *steerVoltSig);
    driveInputs[i] = driveVoltSig->GetValue();
    steerInputs[i] = steerVoltSig->GetValue();
  }

  Eigen::Matrix<double, 2, 2> driveDiscA;
  Eigen::Matrix<double, 2, 1> driveDiscB;
  frc::DiscretizeAB<2, 1>(
    drivePlant.A(), drivePlant.B(), deltaTime, &driveDiscA, &driveDiscB);

  Eigen::Matrix<double, 2, 2> steerDiscA;
  Eigen::Matrix<double, 2, 1> steerDiscB;
  frc::DiscretizeAB<2, 1>(
    steerPlant.A(), steerPlant.B(), deltaTime, &steerDiscA, &steerDiscB);

  std::array<frc::SwerveModulePosition, 4> moduleDeltas;

  for (int i = 0; i < 4; i++) {
    double prevDriveStatePos = driveStates[i](0, 0);
    driveStates[i] = CalculateX(driveDiscA, driveDiscB, driveStates[i],
      driveInputs[i], units::volt_t{constants::drivebase::gains::DRIVE_KS});
    double currentDriveStatePos = driveStates[i](0, 0);
    steerStates[i] = CalculateX(steerDiscA, steerDiscB, steerStates[i],
      steerInputs[i], units::volt_t{constants::drivebase::gains::STEER_KS});
    double currentSteerStatePos = steerStates[i](0, 0);
    moduleDeltas[i] = frc::SwerveModulePosition{
      units::meter_t{currentDriveStatePos - prevDriveStatePos},
      frc::Rotation2d{units::radian_t{currentSteerStatePos}}};
  }

  frc::Twist2d twist = kinematics.ToTwist2d(moduleDeltas);
  lastAngle = lastAngle + frc::Rotation2d{twist.dtheta};
  pose = pose.Exp(twist);
  omega = twist.dtheta / deltaTime;
  pigeonSim.SetRawYaw(lastAngle.Degrees());

  auto driveState = GetDriveStates();
  auto steerState = GetSteerStates();

  for (int i = 0; i < 4; i++) {
    ctre::phoenix6::sim::TalonFXSimState& steerMotor{
      modulesToApply[i].steerMotor.GetSimState()};
    ctre::phoenix6::sim::TalonFXSimState& driveMotor{
      modulesToApply[i].driveMotor.GetSimState()};
    ctre::phoenix6::sim::CANcoderSimState& encoder{
      modulesToApply[i].steerEnc.GetSimState()};

    steerMotor.SetRawRotorPosition(units::radian_t{steerState[i](0, 0)}
      * constants::drivebase::physical::STEER_GEARING);
    steerMotor.SetRotorVelocity(units::radians_per_second_t{steerState[i](1, 0)}
      * constants::drivebase::physical::STEER_GEARING);
    steerMotor.SetSupplyVoltage(supplyVoltage);

    encoder.SetRawPosition(units::radian_t{steerState[i](0, 0)});
    encoder.SetVelocity(units::radians_per_second_t{steerState[i](1, 0)});
    encoder.SetSupplyVoltage(supplyVoltage);

    driveMotor.SetRawRotorPosition(units::radian_t{driveState[i](0, 0)}
      * constants::drivebase::physical::DRIVE_GEARING
      / (constants::drivebase::physical::WHEEL_DIAM / 2).to<double>());
    driveMotor.SetRotorVelocity(
      units::radians_per_second_t{driveStates[i](1, 0)}
      * constants::drivebase::physical::DRIVE_GEARING
      / (constants::drivebase::physical::WHEEL_DIAM / 2).to<double>());
    driveMotor.SetSupplyVoltage(supplyVoltage);
  }
}
