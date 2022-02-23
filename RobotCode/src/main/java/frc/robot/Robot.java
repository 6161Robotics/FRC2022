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
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private XboxController _controller;
  

  //private final MotorController m_leftMotor = new PWMSparkMax(0);
  //private final MotorController m_rightMotor = new PWMSparkMax(1);

  /* Master Talons for arcade drive */
  // Swapped CAN IDs to get it to drive the right direction
  CANSparkMax _frontLeftSpark = new CANSparkMax(22, MotorType.kBrushless);
  CANSparkMax _frontRightSpark = new CANSparkMax(21, MotorType.kBrushless);

	//WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(1);
	//WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(2);

	/* Follower Talons + Victors for six motor drives */
	CANSparkMax _rearLeftSpark = new CANSparkMax(23, MotorType.kBrushless);
  CANSparkMax _rearRightSpark = new CANSparkMax(24, MotorType.kBrushless);
  MotorControllerGroup m_left = new MotorControllerGroup(_frontLeftSpark, _rearLeftSpark);
  MotorControllerGroup m_right = new MotorControllerGroup(_frontRightSpark, _rearRightSpark);
  //WPI_TalonSRX _leftFollower1 = new WPI_TalonSRX(5);
	//WPI_VictorSPX _rightFollower1 = new WPI_VictorSPX(7);
	//WPI_TalonSRX _leftFollower2 = new WPI_TalonSRX(4);
	//WPI_VictorSPX _rightFollower2 = new WPI_VictorSPX(17);
  
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_right.setInverted(true);

    m_myRobot = new DifferentialDrive(m_left, m_right);
    //m_leftStick = new Joystick(0);
    //m_rightStick = new Joystick(1);
    _controller = new XboxController(0);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(_controller.getLeftY(), _controller.getRightY());
  }
}
