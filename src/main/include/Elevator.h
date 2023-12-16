#include <rev/CANSparkMax.h>
#include <frc/XboxController.h>

// https://prod.liveshare.vsengsaas.visualstudio.com/join?C47C0D76BBF094EB3811E324BE0B7DC4892F

struct ElevatorConfig{

    double deadzone; 
    double slowMultiplier;
    double fastMultiplier;


};



class Elevator{
    public:
        Elevator() = default;
        void Configure(ElevatorConfig config);
        void ElevatorOperation(double controllerSpeed, bool slowMode);
    private:
        rev::CANSparkMax m_elevatorMotor{9, rev::CANSparkMax::MotorType::kBrushless};
        double m_deadzone;
        double m_slowMultplier;
        double m_fastMultplier;
};