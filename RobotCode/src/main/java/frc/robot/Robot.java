// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  // The drive base object
  private DifferentialDrive m_myRobot;
  // The Drivers Controller object
  private XboxController m_driverController;
  
  // The Left Side Motor Controllers
  CANSparkMax m_frontLeftSpark;
	CANSparkMax m_rearLeftSpark;
  // The Right Side Motor Controllers
  CANSparkMax m_frontRightSpark;
  CANSparkMax m_rearRightSpark;

  VictorSPX deliveryMotor;
  VictorSPX liftMotor;
  // Group the motor controllers per side
  MotorControllerGroup m_left; 
  MotorControllerGroup m_right;

  /**
   * This code is called when the robot starts to execute
   */
  @Override
  public void robotInit() {
    // The Left Side Motor Controllers
    m_frontLeftSpark = new CANSparkMax(21, MotorType.kBrushless);
	  m_rearLeftSpark = new CANSparkMax(23, MotorType.kBrushless);
    //m_rearLeftSpark.follow(m_frontLeftSpark);
    // The Right Side Motor Controllers
    m_frontRightSpark = new CANSparkMax(24, MotorType.kBrushless);
    m_rearRightSpark = new CANSparkMax(22, MotorType.kBrushless);
    deliveryMotor = new VictorSPX(10);
    liftMotor = new VictorSPX(11);
    //m_rearRightSpark.follow(m_frontLeftSpark);
    // Group the motor controllers per side
    m_left = new MotorControllerGroup(m_frontLeftSpark, m_rearLeftSpark);
    m_right = new MotorControllerGroup(m_frontRightSpark, m_rearRightSpark);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_right.setInverted(true);
    // Setup Motor Followers

    // Populate the drive base object
    m_myRobot = new DifferentialDrive(m_left, m_right);
    // Populate the driver controller object
    m_driverController = new XboxController(0);
  }

  /**
   * This method is called on a loop during the teleop time
   */
  @Override
  public void teleopPeriodic() {
    // Invert to make it drive the right direction
    // 11 CAN is pull hook Down
    // 10 CAN is deliver hook
    // Debounce Thumb Sticks
    
    // Allow the motor to be run in two directions
    if (m_driverController.getBButton()){
      deliveryMotor.set(ControlMode.PercentOutput, 1);
    } else if (m_driverController.getAButton()) {
      deliveryMotor.set(ControlMode.PercentOutput, -1);
    } else {
      deliveryMotor.set(ControlMode.PercentOutput, 0);
    }

    // Allow the motor to be run in two directions
    if (m_driverController.getYButton()){
      liftMotor.set(ControlMode.PercentOutput, 1);
    } else if (m_driverController.getXButton()) {
      liftMotor.set(ControlMode.PercentOutput, -1);
    } else {
      liftMotor.set(ControlMode.PercentOutput, 0);
    }

    // roll off low inputs
    double leftStick = 0;
    double rightStick = 0;   
    if (-0.05 < m_driverController.getLeftY()
    && m_driverController.getLeftY() < 0.05){
      leftStick = 0;
    } else {
      leftStick = m_driverController.getLeftY() * -1;
    }

    if (-0.05 < m_driverController.getRightY()
    && m_driverController.getRightY() < 0.05){
      rightStick = 0;
    } else {
      rightStick = m_driverController.getRightY() * -1;
    }
    //
    m_myRobot.tankDrive(leftStick, rightStick);
  }
}
