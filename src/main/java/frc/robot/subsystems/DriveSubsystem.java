// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.libraries.ConsoleJoystick;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new Drive. */
  private ConsoleJoystick m_console;
  private CANSparkMax m_leftController1;
  private CANSparkMax m_leftController2;
  private CANSparkMax m_rightController1;
  private CANSparkMax m_rightController2;
  private DifferentialDrive  m_differentialdrive;

  public DriveSubsystem() {
    m_leftController1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
    m_leftController2 = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
    m_rightController1 = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
    m_rightController2 = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

    m_rightController1.restoreFactoryDefaults();
    m_rightController2.restoreFactoryDefaults();
    m_leftController1.restoreFactoryDefaults();
    m_leftController2.restoreFactoryDefaults();

    m_leftController1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_leftController2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightController1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightController2.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_leftController1.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);
    m_leftController2.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);
    m_rightController1.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);
    m_rightController2.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);

    m_leftController1.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMT);
    m_leftController2.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMT);
    m_rightController1.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMT);
    m_rightController2.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMT);
    
    m_leftController2.follow(m_leftController1);
    m_rightController2.follow(m_rightController1);
    m_differentialdrive = new DifferentialDrive(m_leftController1, m_rightController1);
    m_rightController1.setInverted(!DriveConstants.kIS_DRIVE_INVERTED);
    m_leftController1.setInverted(DriveConstants.kIS_DRIVE_INVERTED);
    // super(
    // The PIDController used by the subsystem
    // new PIDController(0, 0, 0));
  }

  @Override
  public void periodic() {
  }

  public void arcadeDrive(double velocity, double heading) {
    m_differentialdrive.arcadeDrive(velocity, -1 * heading);
    
    // System.out.println("velocity="+velocity);
    // System.out.println("heading="+heading);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

}
