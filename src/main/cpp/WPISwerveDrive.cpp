#include "WPISwerveDrive.h"

void WPISwerveDrive::Configure(SwerveConfig &config){
    m_ebrake = config.ebrake;
    m_maxDriveSpeed = config.maxDriveSpeed;
    m_maxTurnSpeed = config.maxTurnSpeed;
    m_orientation = config.orientation;
    frc::Translation2d m_frontLeftLocation = config.frontLeftLocation;
    frc::Translation2d m_frontRightLocation = config.frontRightLocation;
    frc::Translation2d m_backLeftLocation = config.backLeftLocation;
    frc::Translation2d m_backRightLocation = config.backRightLocation;
    SetIdleMode(config.idle_mode);
}


bool WPISwerveDrive::GetEbrake() {
    return m_ebrake;
}
void WPISwerveDrive::SetEbrake(bool ebrake) {
    m_ebrake = ebrake;
}
void WPISwerveDrive::Drive(double x_position, double y_position, double rotation) {
    

     Drive(frc::ChassisSpeeds((units::feet_per_second_t)x_position, (units::feet_per_second_t)y_position, (units::degrees_per_second_t)rotation));

}
void WPISwerveDrive::Drive(units::feet_per_second_t vx, units::feet_per_second_t vy, units::degrees_per_second_t omega) {
   
    Drive(frc::ChassisSpeeds{vx, vy, omega});
    

}
void WPISwerveDrive::Drive(frc::ChassisSpeeds speed) {


    frc::SwerveDriveKinematics<4> m_kinematics{
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
    m_backRightLocation};
 

    // states = m_kinematics.ToSwerveModuleStates(speed);
    auto states = m_kinematics.ToSwerveModuleStates(speed);
    
    m_kinematics.DesaturateWheelSpeeds(&states, units::feet_per_second_t(m_maxDriveSpeed));


    std::vector<frc::SwerveModuleState> stateN;
    stateN.push_back(states[0]);
    stateN.push_back(states[1]);
    stateN.push_back(states[2]);
    stateN.push_back(states[3]);

    Drive(stateN);

}
void WPISwerveDrive::Drive(std::vector<frc::SwerveModuleState> &state) {
    if (!m_ebrake) {
        for(int i = 0; i < state.size(); i++){

            m_modules[i].SetState(state[i]);

        }
    }
} 
bool WPISwerveDrive::GetIdleMode() {
    return false;
}
void WPISwerveDrive::SetIdleMode(bool idle_mode) {
     for(int i = 0; i < m_modules.size(); i++){

        m_modules[i].SetIdleMode(idle_mode);

    }
}
void WPISwerveDrive::SetRobotOriented() {
    m_orientation = false;
}
void WPISwerveDrive::SetFieldOriented() {
    m_orientation = true; 
}
bool WPISwerveDrive::GetOrientedMode() {
    return m_orientation;
}