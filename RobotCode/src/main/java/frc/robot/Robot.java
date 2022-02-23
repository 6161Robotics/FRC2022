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
import com.ctre.phoenix.motorcontrol.can.*;
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
  CANSparkMax m_frontLeftSpark = new CANSparkMax(22, MotorType.kBrushless);
	CANSparkMax m_rearLeftSpark = new CANSparkMax(23, MotorType.kBrushless);
  // The Right Side Motor Controllers
  CANSparkMax m_frontRightSpark = new CANSparkMax(21, MotorType.kBrushless);
  CANSparkMax m_rearRightSpark = new CANSparkMax(24, MotorType.kBrushless);
  // Group the motor controllers per side
  MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeftSpark, m_rearLeftSpark);
  MotorControllerGroup m_right = new MotorControllerGroup(m_frontRightSpark, m_rearRightSpark);

  /**
   * This code is called when the robot starts to execute
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_right.setInverted(true);
    // Populate the drive base object
    m_myRobot = new DifferentialDrive(m_left, m_right);
    // Populate the driver controller object
    m_driverController = new XboxController(0);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_driverController.getLeftY(), m_driverController.getRightY());
  }
}
