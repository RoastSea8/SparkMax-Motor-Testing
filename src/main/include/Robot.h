/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/TimedRobot.h>
#include "rev/CANSparkMax.h"
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <string>
#include <vector>

class Robot : public frc::TimedRobot {
 public:
  static const int rightLeadDeviceID = 1, rightFollowDeviceID = 2, leftLeadDeviceID = 3, leftFollowDeviceID = 4;
  rev::CANSparkMax* m_leftLeadMotor;
  rev::CANSparkMax* m_rightLeadMotor;
  rev::CANSparkMax* m_leftFollowMotor;
  rev::CANSparkMax* m_rightFollowMotor;

  double leftMotorSpeed{0.0}, rightMotorSpeed{0.0}, leftMotorVelocity{0.0}, rightMotorVelocity{0.0};
  enum Direction {F, B, L, R};
  std::vector<std::string> d2String {"forward", "backward", "left", "right"};
  Direction d;

  frc::DifferentialDrive* m_robotDrive;

  frc::XboxController* m_controller;

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

  bool inRange(double);
  void Move(double, double);
};

bool Robot::inRange(double speed) {
  if ((speed >= -1) && (speed <= 1)) {
    return true;
  }
  else return false;
}

void Robot::Move(double leftMotorSpeed, double rightMotorSpeed) {
  m_robotDrive->TankDrive(leftMotorSpeed, rightMotorSpeed);

  if (inRange(leftMotorSpeed) && inRange(rightMotorSpeed)) {
    if (leftMotorSpeed == rightMotorSpeed) {
      if (leftMotorSpeed > 0) d = F; else d = B;
    }
    else {
      if (leftMotorSpeed < rightMotorSpeed) d = L; else d = R;
    }
    
    wpi::outs() << "\n Expected direction: " << d2String[d] << ", Left motor speed: " << leftMotorSpeed << ", Right motor speed: " << rightMotorSpeed << "\n";

    leftMotorVelocity = m_leftLeadMotor->GetEncoder().GetVelocity();
    rightMotorVelocity = m_rightLeadMotor->GetEncoder().GetVelocity();

    switch (d) {
      case F:
        // if the velocities of both motors are the same, and if their velocities are greater than 0, then the robot is moving forward
        if((leftMotorVelocity == rightMotorVelocity) && (leftMotorVelocity > 0))
          wpi::outs() << "SUCCESSFUL: Robot moving FORWARD with LEFT motor velocity = " << leftMotorVelocity << " and RIGHT motor velocity = " << rightMotorVelocity <<"\n";
        else
        {
          wpi::outs() << "ERROR: Robot not moving FORWARD - LEFT motor velocity = " << leftMotorVelocity << " and RIGHT motor velocity = " << rightMotorVelocity << "\n";
          return;
        }
        break;
      case B: 
        // if the velocities of both motors are the same, and if their velocities are less than 0, then the robot is moving backward
        if((leftMotorVelocity == rightMotorVelocity) && (leftMotorVelocity < 0))
          wpi::outs() << "SUCCESSFULL: Robot moving BACKWARD with LEFT motor velocity = " << leftMotorVelocity << " and RIGHT motor velocity = " << rightMotorVelocity << "\n";
        else {
          wpi::outs() << "ERROR: Robot not moving BACKWARD - LEFT motor velocity = " << leftMotorVelocity << " and RIGHT motor velocity = " << rightMotorVelocity << "\n";
        }
        break;
      case L: 
        // if the velocity of the right motor is greater than the velocity of the left motor, the robot is turning left
        if (leftMotorVelocity < rightMotorVelocity)
          wpi::outs() << "SUCCESSFULL: Robot turning LEFT with LEFT motor velocity = " << leftMotorVelocity << " and RIGHT motor velocity = " << rightMotorVelocity << "\n";
        else {
          wpi::outs() << "ERROR: Robot not turning LEFT - LEFT motor velocity = " << leftMotorVelocity << " and RIGHT motor velocity = " << rightMotorVelocity << "\n";
        }
        break;
      case R:
        // if the velocity of the left motor is greater than the velocity of the right motor, the robot is turning right
        if (leftMotorVelocity > rightMotorVelocity)
          wpi::outs() << "SUCCESSFULL: Robot turning RIGHT with LEFT motor velocity = " << leftMotorVelocity << " and RIGHT motor velocity = " << rightMotorVelocity << "\n";
        else {
          wpi::outs() << "ERROR: Robot not turning RIGHT - LEFT motor velocity = " << leftMotorVelocity << " and RIGHT motor velocity = " << rightMotorVelocity << "\n";
        }
        break;
        default:
        //Error invalid direction 
        wpi::outs() << "ERROR: Invalid Direction \n";
      }
  } else { 
    // outputting an error if the left or right motor speeds are not in range
    wpi::outs() << "ERROR: Left motor speed and right motor speed not in range \n";
  }
}