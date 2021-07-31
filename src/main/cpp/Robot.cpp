/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "rev/CANSparkMax.h"

void Robot::RobotInit() {
  m_leftLeadMotor->RestoreFactoryDefaults();
  m_rightLeadMotor->RestoreFactoryDefaults();
  m_leftFollowMotor->RestoreFactoryDefaults();
  m_rightFollowMotor->RestoreFactoryDefaults();
  
  m_leftLeadMotor->SetInverted(false);
  m_leftFollowMotor->Follow(*m_leftLeadMotor);
  m_rightLeadMotor->SetInverted(true);
  m_rightFollowMotor->Follow(*m_rightLeadMotor);

  m_robotDrive = new frc::DifferentialDrive(*m_leftLeadMotor, *m_rightLeadMotor);

  m_controller = new frc::XboxController{0}; // replace with USB port num on DS
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  if (m_controller->GetAButton())
    Move(0.2, 0.2);   // Forward check
  else if (m_controller->GetBButton())
    Move(-0.2, -0.2); // Backward check
  else if (m_controller->GetYButton())
    Move(-0.2, 0.2);  // Left check
  else if (m_controller->GetXButton())
    Move(0.2, -0.2);  // Right check
  else if (m_controller->GetBumper(frc::GenericHID::kRightHand))
    m_robotDrive->StopMotor();
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif