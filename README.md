[![Build Robot Code](https://github.com/frc2053/testdrivebase/actions/workflows/build.yml/badge.svg)](https://github.com/frc2053/testdrivebase/actions/workflows/build.yml)
[![Formatting](https://github.com/frc2053/testdrivebase/actions/workflows/format.yml/badge.svg)](https://github.com/frc2053/testdrivebase/actions/workflows/format.yml)

# FRC 2053 Southern Tier Robotics Test Drivebase Code

This repo holds our offseason test drivebase code. Our test drivebase is a swerve drivetrain using V3 Falcon 500's with the MK4i modules geared at the "L2" option. We have a pigeon v2 as our IMU as well all running through the CANivore. Our library uses Phoenix V6 Pro.

# How to setup

## Prerequisites
- Python 3 (not installed from Windows store. Please install from Python website)
- Version [2024.1.1 Beta 2](https://github.com/wpilibsuite/allwpilib/releases/tag/v2024.1.1-beta-2) of the WPILib installer installed.
- Visual Studio 2022 with C++ workload

## Install steps: 
- Create your virtual environment
    Inside the root of the project run:
    `python -m venv ./venv`
- Activate your virtual environment
    `./venv/Scripts/Activate.ps1`
- Install the code formatter
    `pip install -r requirements.txt`
- Install the pre-commit hooks for formatting
    `pre-commit install`
- Install these vscode extensions:
    [Python](vscode:extension/ms-python.python)
    [Run On Save](vscode:extension/emeraldwalk.RunOnSave)

## Build steps:
- To build the code, press Ctrl+Shift+P and search for "Build Robot Code". This will build the robot code for the robot, as well as simulation (desktop).

## To run:
- To run in simulation, press Ctrl+Shift+P and search for "Simulate Robot Code". This will launch the simulation.
- To run the code on the robot, press Ctrl+Shift+P and search for "Deploy Robot Code". This will search for a RoboRio over USB or the network and upload the code to the robot.