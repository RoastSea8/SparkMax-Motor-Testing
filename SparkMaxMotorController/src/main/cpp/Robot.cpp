/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "rev/CANSparkMax.h"
#include <iostream> 
#include <fstream>

using namespace std; 

// Create a file called myFile
static ofstream adityaFile;

class Robot : public frc::TimedRobot {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type is set to Brushless motor.
   * 
   * Initializes four brushless motors with CAN IDs 1, 2, 3 and 4.
   */
  static const int leftLeadDeviceID = 1, leftFollowDeviceID = 2, rightLeadDeviceID = 3, rightFollowDeviceID = 4;
  rev::CANSparkMax* m_leftLeadMotor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightLeadMotor = new rev::CANSparkMax(rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_leftFollowMotor = new rev::CANSparkMax(leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightFollowMotor = new rev::CANSparkMax(rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);

  /**
   * A CANAnalog object is constructed using the GetAnalog() method on an 
   * existing CANSparkMax object. 
   */
  rev::CANEncoder m_encoderSensor_left_motor = m_leftLeadMotor->GetEncoder();
  rev::CANEncoder m_encoderSensor_right_motor = m_rightLeadMotor->GetEncoder();

  /**
   * In RobotInit() below, we will configure m_leftFollowMotor and m_rightFollowMotor to follow 
   * m_leftLeadMotor and m_rightLeadMotor, respectively. 
   * 
   * Because of this, we only need to pass the lead motors to m_robotDrive. Whatever commands are 
   * sent to them will automatically be copied by the follower motors
   */
  
  void Move(double leftMotorSpeed, double rightMotorSpeed) {
    int direction; 
    // checking if both motor speeds are in range: between or equal to -1 and 1
    if (((leftMotorSpeed >= -1) && (leftMotorSpeed <= 1)) && ((rightMotorSpeed >= -1) && (rightMotorSpeed <= 1))) {
      if (leftMotorSpeed == rightMotorSpeed){
        if (leftMotorSpeed > 0) {
          direction = 1; //forward
        } else if (leftMotorSpeed < 0) {
          direction = 2; //backward
        }
      } 
      else {
        if (leftMotorSpeed > rightMotorSpeed) {
          direction = 4; //right
        } else {
          direction = 3; //left
        }
      }
      // setting the speed of the lead motors
      m_leftLeadMotor->Set(leftMotorSpeed);
      m_rightLeadMotor->Set(rightMotorSpeed); 
      // outputting speeds of both motors to file 
      adityaFile << "\n Expected direction: " << direction << " Left motor speed: " << leftMotorSpeed << " Right motor speed: " << rightMotorSpeed << "\n";

      // variable to hold the value of GetVelocity() for the left motor
      double leftMotorVelocity = m_encoderSensor_left_motor.GetVelocity();
      // variable to hold the value of GetVelocity() for the right motor
      double rightMotorVelocity = m_encoderSensor_right_motor.GetVelocity();

      switch (direction) {
        case 1:
          // if the velocities of both motors are the same, and if their velocities are greater than 0, then the robot is moving forward
          if((leftMotorVelocity > 0) && (leftMotorVelocity == rightMotorVelocity)) {  
            adityaFile << "SUCCESSFULL: Robot moving FORWARD with LEFT motor velocity = " << leftMotorVelocity << " and RIGHT motor velocity = " << rightMotorVelocity <<"\n";
          } else
          {
            adityaFile << "ERROR: Robot not moving FORWARD - LEFT motor velocity = " << leftMotorVelocity << " and RIGHT motor velocity = " << rightMotorVelocity << "\n";
            return;
          }
          break;
        case 2: 
          // if the velocities of both motors are the same, and if their velocities are less than 0, then the robot is moving backward
          if((leftMotorVelocity < 0) && (leftMotorVelocity == rightMotorVelocity)) {
            adityaFile << "SUCCESSFULL: Robot moving BACKWARD with LEFT motor velocity = " << leftMotorVelocity << " and RIGHT motor velocity = " << rightMotorVelocity << "\n";
          } else {
            adityaFile << "ERROR: Robot not moving BACKWARD - LEFT motor velocity = " << leftMotorVelocity << " and RIGHT motor velocity = " << rightMotorVelocity << "\n";
          }
          break;
        case 3: 
          // if the velocity of the right motor is greater than the velocity of the left motor, the robot is turning left
          if (leftMotorVelocity < rightMotorVelocity) {
            adityaFile << "SUCCESSFULL: Robot turning LEFT with LEFT motor velocity = " << leftMotorVelocity << " and RIGHT motor velocity = " << rightMotorVelocity << "\n";
          } else {
            adityaFile << "ERROR: Robot not turning LEFT - LEFT motor velocity = " << leftMotorVelocity << " and RIGHT motor velocity = " << rightMotorVelocity << "\n";
          }
          break;
        case 4:
          // if the velocity of the left motor is greater than the velocity of the right motor, the robot is turning right
          if (leftMotorVelocity > rightMotorVelocity) {
            adityaFile << "SUCCESSFULL: Robot turning RIGHT with LEFT motor velocity = " << leftMotorVelocity << " and RIGHT motor velocity = " << rightMotorVelocity << "\n";
          } else {
            adityaFile << "ERROR: Robot not turning RIGHT - LEFT motor velocity = " << leftMotorVelocity << " and RIGHT motor velocity = " << rightMotorVelocity << "\n";
          }
          break;
          default:
          //Error invalid direction 
          frc::SmartDashboard::PutString("ERROR: Invalid Direction", "\n");
        }
    } else { 
      // outputting an error if the left or right motor speeds are not in range
      frc::SmartDashboard::PutString("ERROR: Left motor speed not in range/", "right motor speed not in range");
    }
    return;
  }

  void StopMotors(){
    //passing 0 as the parameter to make the motors stop
    m_leftLeadMotor->Set(0);
    m_rightLeadMotor->Set(0);
  } 
 public:
  void RobotInit() {
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftLeadMotor->RestoreFactoryDefaults();
    m_rightLeadMotor->RestoreFactoryDefaults();
    m_leftFollowMotor->RestoreFactoryDefaults();
    m_rightFollowMotor->RestoreFactoryDefaults();
    
    /**
     * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling
     * the Follow() method on the SPARK MAX you want to configure as a follower, and by passing
     * as a parameter the SPARK MAX you want to configure as a leader.
     * 
     * one motor on each side of our drive train is configured to follow a lead motor.
     */
    m_leftFollowMotor->Follow(*m_leftLeadMotor);
    m_rightFollowMotor->Follow(*m_rightLeadMotor);

    adityaFile.open("velocityData.txt");
    //moving at half speed
    Move(0.5, 0.5);   //Forward
    Move(-0.5, -0.5); //Backward
    Move(-0.5, 0.5);  //Left - the left motor moves backward and the right motor moves forward
    Move(0.5, -0.5);  //Right - the left motor moves forward and the right motor moves backward
    StopMotors(); //stopping the motors

    //closing the file
    adityaFile.close();
  };

  void TeleopPeriodic() {

  };

};
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif