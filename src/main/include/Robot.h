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
  
 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;


  frc::XboxController m_pilot1{1};
  Elevator m_elevator;
  
  AnalogAbsoluteEncoder m_back_left_analog_encoder;
  NeoTurnMotor m_back_left_turn_motor;
  NavXGyro gyro;
  AnalogAbsoluteEncoder m_ABSencoder;
  NeoTurnMotor m_turnMotor;
  NeoDriveMotor m_driveMotor;
  WPISwerveModule m_swerveModule_BL;
};