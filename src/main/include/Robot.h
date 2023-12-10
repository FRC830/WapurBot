// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "AnalogAbsoluteEncoder.h"
#include "NeoTurnMotor.h"
#include "NeoDriveMotor.h"
#include "WPISwerveModule.h"
#include "WPISwerveDrive.h"
#include <iostream>
#include "SwerveConfig.h"
#include "NavXGyro.h"
#include "Elevator.h"


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
  void SwerveInit(); 
  
 private:
  static const int NUM_MODULES = 4;

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  
  std::array<AnalogAbsoluteEncoder, NUM_MODULES> _abs_encoders;
  std::array<NeoTurnMotor, NUM_MODULES> _turn_motors;
  std::array<NeoDriveMotor, NUM_MODULES> _drive_motors;
  std::array<WPISwerveModule, NUM_MODULES> _modules;
  WPISwerveDrive _swerve;
  frc::XboxController _xbox_controller{0};

  
  NavXGyro _gyro;
  int m_state;
  std::vector<pathplanner::PathPlannerTrajectory> m_trajectory;
  std::unique_ptr<frc2::CommandPtr> m_command_ptr;
};
