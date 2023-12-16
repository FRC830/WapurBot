// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

  namespace ModulePosition
  { 
    static const int FL = 0;
    static const int FR = 1;
    static const int BL  = 2;
    static const int BR  = 3; 
  };

void Robot::SwerveInit(){
  AbsoluteEncoderConfig abs_config;

  abs_config.encoder = &fl_abs_enc;
  abs_config.is_inverted = FL_ABS_ENC_INVERTED;
  abs_config.zero_heading = FL_ZERO_HEADING;
  _abs_encoders[ModulePosition::FL].Configure(abs_config);

  abs_config.encoder = &fr_abs_enc;
  abs_config.is_inverted = FR_ABS_ENC_INVERTED;
  abs_config.zero_heading = FR_ZERO_HEADING;
  _abs_encoders[ModulePosition::FR].Configure(abs_config);

  abs_config.encoder = &bl_abs_enc;
  abs_config.is_inverted = BL_ABS_ENC_INVERTED;
  abs_config.zero_heading = BL_ZERO_HEADING;
  _abs_encoders[ModulePosition::BL].Configure(abs_config);

  abs_config.encoder = &br_abs_enc;
  abs_config.is_inverted = BR_ABS_ENC_INVERTED;
  abs_config.zero_heading = BR_ZERO_HEADING;
  _abs_encoders[ModulePosition::BR].Configure(abs_config);


  SwerveTurnMotorConfig turn_config;
  turn_config.p=TURN_P;  
  turn_config.i=TURN_I;
  turn_config.d=TURN_D;
  turn_config.ff=TURN_FF;
  turn_config.ratio=MOTOR_ROT_TO_DEG;

  turn_config.deviceID=FL_TURN_MTR_ID;
  turn_config.absouluteEncoder=&_abs_encoders[ModulePosition::FL];
  turn_config.inverted=FL_TURN_MTR_INVERTED;
  turn_config.PID=&fl_turn_pid;
  turn_config.relative_Encoder=&fl_turn_enc;
  turn_config.turn_motor=&fl_turn_mtr;
  _turn_motors[ModulePosition::FL].Configure(turn_config);

  turn_config.deviceID=FR_TURN_MTR_ID;
  turn_config.absouluteEncoder=&_abs_encoders[ModulePosition::FR];
  turn_config.inverted=FR_TURN_MTR_INVERTED;
  turn_config.PID=&fr_turn_pid;
  turn_config.relative_Encoder=&fr_turn_enc;
  turn_config.turn_motor=&fr_turn_mtr;
  _turn_motors[ModulePosition::FR].Configure(turn_config);

  turn_config.deviceID=BL_TURN_MTR_ID;
  turn_config.absouluteEncoder=&_abs_encoders[ModulePosition::BL];
  turn_config.inverted=BL_TURN_MTR_INVERTED;
  turn_config.PID=&bl_turn_pid;
  turn_config.relative_Encoder=&bl_turn_enc;
  turn_config.turn_motor=&bl_turn_mtr;
  _turn_motors[ModulePosition::BL].Configure(turn_config);

  turn_config.deviceID=BR_TURN_MTR_ID;
  turn_config.absouluteEncoder=&_abs_encoders[ModulePosition::BR];
  turn_config.inverted=BR_TURN_MTR_INVERTED;
  turn_config.PID=&br_turn_pid;
  turn_config.relative_Encoder=&br_turn_enc;
  turn_config.turn_motor=&br_turn_mtr;
  _turn_motors[ModulePosition::BR].Configure(turn_config);


  SwerveDriveMotorConfig drive_config;
  drive_config.p = DRIVE_P;
  drive_config.i = DRIVE_I;
  drive_config.d = DRIVE_D;
  drive_config.ff = DRIVE_FF;
  drive_config.ratio = MOTOR_ROT_TO_FT / 60.0;
  
  drive_config.PID = &fl_drive_pid;
  drive_config.encoder = &fl_drive_enc;
  drive_config.motor = &fl_drive_mtr;
  drive_config.correction_factor = FL_POSITION_CORRECTION_FACTOR;
  _drive_motors[ModulePosition::FL].Configure(drive_config);

  drive_config.PID = &fr_drive_pid;
  drive_config.encoder = &fr_drive_enc;
  drive_config.motor = &fr_drive_mtr;
  drive_config.correction_factor = FR_POSITION_CORRECTION_FACTOR;
  _drive_motors[ModulePosition::FR].Configure(drive_config);

  drive_config.PID = &bl_drive_pid;
  drive_config.encoder = &bl_drive_enc;
  drive_config.motor = &bl_drive_mtr;
  drive_config.correction_factor = BL_POSITION_CORRECTION_FACTOR;
  _drive_motors[ModulePosition::BL].Configure(drive_config);

  drive_config.PID = &br_drive_pid;
  drive_config.encoder = &br_drive_enc;
  drive_config.motor = &br_drive_mtr;
  drive_config.correction_factor = BR_POSITION_CORRECTION_FACTOR;
  _drive_motors[ModulePosition::BR].Configure(drive_config);


  SwerveModuleConfig module_config;
  module_config.idleMode=true;

  for (int i = 0; i < NUM_MODULES; ++i)
  {
    module_config.driveMotor = &_drive_motors[i];
    module_config.turnMotor  = &_turn_motors[i];
    _modules[i].Configure(module_config);
  }
  
  GyroConfig gyroConfig;
  gyroConfig.is_inverted = GYRO_INVERTED;
  gyroConfig.zero_heading = GYRO_ZERO_HEADING;
  _gyro.Configure(gyroConfig);

  SwerveConfig swerveConfig;
  swerveConfig.backLeftLocation=bl_position;
  swerveConfig.backRightLocation=br_position;
  swerveConfig.ebrake=false;
  swerveConfig.frontLeftLocation=fl_position;
  swerveConfig.frontRightLocation=fr_position;
  swerveConfig.idle_mode=IS_DRIVE_IN_COAST;
  swerveConfig.maxDriveSpeed=MAX_DRIVE_SPEED_FPS;
  swerveConfig.maxTurnSpeed=MAX_ANGULAR_VELOCITY_DEGPS;
  swerveConfig.orientation=IS_ROBOT_ORIENTED_DRIVE;
  swerveConfig.deadzone=CONTROLLER_DEADZONE;
  swerveConfig.gyro=&_gyro;
  auto* temp = _swerve.GetModules();
  for (int i = 0; i < 4; ++i)
  {
    (*temp)[i] = &_modules[i];
  }
  _swerve.Configure(swerveConfig);
  _swerve.SetFieldOriented();
}

void Robot::RobotInit() {
  SwerveInit();

  //elevator config

  ElevatorConfig eleConfig;

  eleConfig.deadzone = 0.05;
  eleConfig.slowMultiplier = 0.1; 
  eleConfig.fastMultiplier = 0.4;


  _elevator.Configure(eleConfig);

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);
  m_trajectory = pathplanner::PathPlanner::loadPathGroup("test_path", {pathplanner::PathConstraints{5_mps, 3.5_mps_sq}});
  std::cout << "Trajectory size: " << m_trajectory.size() << std::endl;
  m_command_ptr = std::make_unique<frc2::CommandPtr>(_swerve.GetAutoBuilder()->fullAuto(m_trajectory));
  // m_commandptr = std::move(_swerve.GetAutoBuilder()->fullAuto(trajectory));
  m_state = 0;
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here)
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  // auto command = _swerve.GetAutoBuilder();
  _swerve.SetRobotOriented();

  switch (m_state)
  {
    case 0:
      
      {
        m_command_ptr->get()->Initialize();
        std::cout << m_state << std::endl;
        ++m_state;
        break;
      }
    case 1:
    {
      m_command_ptr->get()->Execute();
      std::cout << m_state << std::endl;
      if (m_command_ptr->get()->IsFinished())
      {
        ++m_state;
      }
      break;
    }
    case 2:
    {
      m_command_ptr->get()->End(false);
      std::cout << m_state << std::endl;
      ++m_state;
      break;
    }
    case 3:
    default:
      break;
  }
}

void Robot::TeleopInit() 
{
  
// deleted old swerve init spam

  frc::SmartDashboard::PutNumber("Speed", 0.0);
  frc::SmartDashboard::PutNumber("ang", 0.0);
  
}

void Robot::TeleopPeriodic() 
{ 
  _swerve.SetFieldOriented();
  // TEMPORARY - for finding conversion factor
  frc::SmartDashboard::PutNumber("FL Drive Motor Rotations", fl_drive_enc.GetPosition());
  frc::SmartDashboard::PutNumber("FR Drive Motor Rotations", fr_drive_enc.GetPosition());
  frc::SmartDashboard::PutNumber("BR Drive Motor Rotations", br_drive_enc.GetPosition());
  frc::SmartDashboard::PutNumber("BL Drive Motor Rotations", bl_drive_enc.GetPosition());

  // TEMPORARY
  frc::SmartDashboard::PutNumber("Pose X", _swerve.GetPose().Translation().X().to<double>());
  frc::SmartDashboard::PutNumber("Pose Y", _swerve.GetPose().Translation().Y().to<double>());
  frc::SmartDashboard::PutNumber("Pose Rotation", _swerve.GetPose().Rotation().Degrees().to<double>());



  // //m_back_left_turn_motor.SetRotation(frc::Rotation2d(static_cast<units::degree_t>(frc::SmartDashboard::GetNumber("backleft turn motor position", 69))));
  // //std::cout << "\rCurrent motor heading/positon:\t" << static_cast<double>(m_back_left_analog_encoder.GetHeading().Degrees()) << "\tsoftware motor positon" << frc::SmartDashboard::GetNumber("backleft turn motor position", 69);
  // double p = frc::SmartDashboard::GetNumber("p", 1);
  // double i = frc::SmartDashboard::GetNumber("i", 1);
  // double d = frc::SmartDashboard::GetNumber("d", 1);
  // double sp = frc::SmartDashboard::GetNumber("Set Point", 0);

  // if (p != bl_turn_pid.GetP() || i != bl_turn_pid.GetI() || d != bl_turn_pid.GetD())
  // {


  //   // frc::SmartDashboard::PutNumber("p", p);
  //   // frc::SmartDashboard::PutNumber("i", i);
  //   // frc::SmartDashboard::PutNumber("d", d);
  // }

  auto speed = frc::SmartDashboard::GetNumber("Speed", 0.0);
  auto angle = frc::SmartDashboard::GetNumber("ang", 0.0);

  frc::SwerveModuleState state{units::feet_per_second_t(speed), units::degree_t(angle)};
  //bl_turn_pid.SetReference(angle, rev::CANSparkMax::ControlType::kPosition);
  //m_swerveModule_BL.SetState(state);
  // for (int i = 0; i < 4; i++) 
  // {
  //   _modules[i].SetState(state);
  // }

  // frc::SmartDashboard::PutNumber("Set Position", sp);

  // Raw heading of absolute encoders
  frc::SmartDashboard::PutNumber("FL Raw Heading", double(_abs_encoders[0].GetRawHeading().Degrees()));
  frc::SmartDashboard::PutNumber("FR Raw Heading", double(_abs_encoders[1].GetRawHeading().Degrees()));
  frc::SmartDashboard::PutNumber("BL Raw Heading", double(_abs_encoders[2].GetRawHeading().Degrees()));
  frc::SmartDashboard::PutNumber("BR Raw Heading", double(_abs_encoders[3].GetRawHeading().Degrees()));
  frc::SmartDashboard::PutNumber("Gyro Heading", _gyro.GetHeading().Degrees().to<double>());

  // Heading of turn motors
  frc::SmartDashboard::PutNumber("FL Heading", fl_turn_enc.GetPosition());
  frc::SmartDashboard::PutNumber("FR Heading", fr_turn_enc.GetPosition());
  frc::SmartDashboard::PutNumber("BL Heading", bl_turn_enc.GetPosition());
  frc::SmartDashboard::PutNumber("BR Heading", br_turn_enc.GetPosition());

  // Velocity of drive motors
  frc::SmartDashboard::PutNumber("FL Velocity", fl_drive_enc.GetVelocity());
  frc::SmartDashboard::PutNumber("FR Velocity", fr_drive_enc.GetVelocity());
  frc::SmartDashboard::PutNumber("BL Velocity", bl_drive_enc.GetVelocity());
  frc::SmartDashboard::PutNumber("BR Velocity", br_drive_enc.GetVelocity());

  // XBox controller
  
  double slowFactor = 0.2;


  bool slowMode = _pilot.GetLeftBumper();
  double left_x = slowMode ? _pilot.GetLeftX() * slowFactor : _pilot.GetLeftX();
  double left_y = slowMode ? _pilot.GetLeftY() * slowFactor : _pilot.GetLeftY();
  double right_x = slowMode ? _pilot.GetRightX() * slowFactor : _pilot.GetRightX();

  

  // TODO: Add deadzone to correct joystick drift
  _swerve.Drive(left_y, left_x, right_x);
  _elevator.ElevatorOperation(_coPilot.GetRightY(), _coPilot.GetLeftBumper());
  
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif