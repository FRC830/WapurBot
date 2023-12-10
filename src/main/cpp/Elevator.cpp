#include "Elevator.h"
//deadzone


void Elevator::ElevatorOperation(double controllerSpeed, bool slowMode) {
  if (slowMode)
  {
      if (controllerSpeed > m_deadzone)
    {
      m_elevatorMotor.Set((-(controllerSpeed) + m_deadzone) / (1 - m_deadzone) * m_multplier);
    }
    else if(controllerSpeed < m_deadzone) {
      m_elevatorMotor.Set((controllerSpeed - m_deadzone)/(1 - m_deadzone) * m_multplier);
    }
  }
  else
  {
    if (controllerSpeed > m_deadzone)
    {
      m_elevatorMotor.Set((-(controllerSpeed) + m_deadzone) / (1 - m_deadzone));
    }
    else if(controllerSpeed < m_deadzone) 
    {
      m_elevatorMotor.Set((controllerSpeed - m_deadzone)/(1 - m_deadzone));
    }
  }
}