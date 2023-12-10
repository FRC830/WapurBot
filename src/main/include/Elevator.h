#include <rev/CANSparkMax.h>
#include <frc/XboxController.h>

// https://prod.liveshare.vsengsaas.visualstudio.com/join?C47C0D76BBF094EB3811E324BE0B7DC4892F

class Elevator{
    public:
        Elevator();
        void ElevatorOperation(double controllerSpeed, bool slowMode);
    private:
        rev::CANSparkMax m_elevatorMotor = rev::CANSparkMax(2, rev::CANSparkMaxLowLevel::MotorType::kBrushless); 
        int m_deadzone = 0.3;
        double m_multplier = 0.2;
};