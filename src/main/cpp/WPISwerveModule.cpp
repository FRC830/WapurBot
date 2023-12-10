#include "WPISwerveModule.h"

void WPISwerveModule::Configure(SwerveModuleConfig &config)
{
    m_driveMotor = config.driveMotor;
    m_turnMotor = config.turnMotor;
    SetIdleMode(config.idleMode);

};

void WPISwerveModule::SetState(frc::SwerveModuleState state)
{
    frc::SwerveModuleState::Optimize(state, m_turnMotor->GetRotation());
    m_turnMotor->SetRotation(state.angle);
    m_driveMotor->SetVelocity(state.speed);
};

frc::SwerveModuleState WPISwerveModule::GetState()
{
 auto angle = m_turnMotor->GetRotation();
 auto speed = m_driveMotor->GetVelocity();
 return frc::SwerveModuleState(units::feet_per_second_t(speed),angle);
};

void WPISwerveModule::SetIdleMode(bool idleMode)
{
    m_idleMode = idleMode;
    m_driveMotor->SetIdleMode(idleMode);
};

bool WPISwerveModule::GetIdleMode()
{
    return m_driveMotor->GetIdleMode();
};
