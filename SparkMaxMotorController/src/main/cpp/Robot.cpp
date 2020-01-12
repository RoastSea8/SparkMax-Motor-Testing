/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "rev/CANSparkMax.h"

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
  rev::CANAnalog m_analogSensor_left_motor = m_leftLeadMotor->GetAnalog();
  rev::CANAnalog m_analogSensor_right_motor = m_rightLeadMotor->GetAnalog();

  /**
   * In RobotInit() below, we will configure m_leftFollowMotor and m_rightFollowMotor to follow 
   * m_leftLeadMotor and m_rightLeadMotor, respectively. 
   * 
   * Because of this, we only need to pass the lead motors to m_robotDrive. Whatever commands are 
   * sent to them will automatically be copied by the follower motors
   */
  void MoveForward(double speed) {
    //speeds are positive so that the motors move forward
    m_leftLeadMotor->Set(speed); 
    m_rightLeadMotor->Set(speed);
  }
  
  void MoveBackward(double speed) {
    //setting the speeds to negative to make the motors move backward
    m_leftLeadMotor->Set(-speed); 
    m_rightLeadMotor->Set(-speed);
  }

  void TurnLeft(double speed){
    //setting the speed to negative to make the left motor move backward
    m_leftLeadMotor->Set(-speed); 
    //setting the speed to positive to make the right motor move forward
    m_rightLeadMotor->Set(speed); 
  }

  void TurnRight(double speed){
    //setting the speed to positive to make the left motor move forward 
    m_leftLeadMotor->Set(speed); 
    //setting the speed to negative to make the right motor move backward
    m_rightLeadMotor->Set(-speed); 
  } 

  void StopMotor(){
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
  }

  void TeleopPeriodic() {
    //setting the speed to 1 so that it moves forward
    MoveForward(1);
    // read the voltage on the lead motors to check whether motors are moving forward or not. 
    if((m_analogSensor_left_motor.GetVelocity() > 0) && (m_analogSensor_right_motor.GetVelocity() > 0)) {  
      frc::SmartDashboard::PutString("Motor Sensor Positive voltage", "Motor moving forward");
    } else { 
      
      frc::SmartDashboard::PutString("ERROR:Motor Sensor voltage", "Not positive");
    }

    //setting the speed to -1 so that it moves backward
    MoveBackward(1);
    // read the voltage on the lead motors to check whether motors are moving backward or not
    if((m_analogSensor_left_motor.GetVelocity() < 0) && (m_analogSensor_right_motor.GetVelocity() < 0)) {  
      frc::SmartDashboard::PutString("Motor Sensor Negative voltage", "Motor moving backward");
    } else {

      frc::SmartDashboard::PutString("ERROR:Motor Sensor voltage", "Not negative");
    }

    TurnLeft(1);
    if ((m_analogSensor_left_motor.GetVelocity() < 0) && (m_analogSensor_right_motor.GetVelocity() > 0)) {
      frc::SmartDashboard::PutString("Left Motor Sensor Negative voltage / Right Motor Sensor Positive voltage", "Motor moving left");
    } else {
      frc::SmartDashboard::PutString("ERROR:Motor Sensor voltage", "Not turning left");
    }

    TurnRight(1);
    if ((m_analogSensor_left_motor.GetVelocity() > 0) && (m_analogSensor_right_motor.GetVelocity() < 0)) {
      frc::SmartDashboard::PutString("Left Motor Sensor Positive voltage / Right Motor Sensor Negative voltage", "Motor moving left");
    } else {
      frc::SmartDashboard::PutString("ERROR:Motor Sensor voltage", "Not turning right");
    }
    
    StopMotor(); //passing 0 as the input parameter to make the speed 0 = stop
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif