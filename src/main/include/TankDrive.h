#include <rev/CANSparkMax.h>
#include <frc/XboxController.h>

class TankDrive{
    public: 
        void Drive();

    private:     
        rev::CANSparkMax mL = rev::CANSparkMax(0, rev::CANSparkMaxLowLevel::MotorType::kBrushless); // motor lefty
        rev::CANSparkMax mR = rev::CANSparkMax(1, rev::CANSparkMaxLowLevel::MotorType::kBrushless); // motor righty
        frc::XboxController pilot{0};
};