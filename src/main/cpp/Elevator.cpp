#include "Elevator.h"
//deadzone

void Elevator::Configure(ElevatorConfig config) {


  m_deadzone = config.deadzone;
  m_slowMultplier = config.slowMultiplier;
  m_fastMultplier = config.fastMultiplier;
  m_elevatorMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

}

void Elevator::ElevatorOperation(double controllerSpeed, bool slowMode) {


  double speed = 0; 

   if (controllerSpeed > m_deadzone)
    {
        speed = (controllerSpeed - m_deadzone) / (1 - m_deadzone);
    }
    else if (controllerSpeed < -(m_deadzone))
    {
        speed = (controllerSpeed + m_deadzone) / (1 - m_deadzone);
    }

    speed *= slowMode ? m_slowMultplier : m_fastMultplier;

  m_elevatorMotor.Set(speed);

}