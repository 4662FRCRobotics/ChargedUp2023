// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private DifferentialDrive m_differentialdrive;
  private double m_throttle;
  private AHRS m_navX;
  private RelativeEncoder m_leftEncoder1;
  private RelativeEncoder m_rightEncoder1;
  private double m_leftEncoderSign;
  private double m_rightEncoderSign;
  private DifferentialDriveOdometry m_diffOdometry;
  private double m_driveDistance;

  public DriveSubsystem() {
    m_throttle= DriveConstants.kMEDIUM_GEAR_SPEED;
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
    m_leftEncoder1 = m_leftController1.getEncoder();
    m_rightEncoder1 = m_rightController1.getEncoder();
    m_diffOdometry = new DifferentialDriveOdometry(new Rotation2d(), m_driveDistance, m_driveDistance);
    m_navX = new AHRS(SerialPort.Port.kMXP);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Yaw", getYaw());
    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Roll", getRoll());
    SmartDashboard.putNumber("Velocity", getVelocity());
    SmartDashboard.putNumber("Left Distance", getLeftDistance());
    SmartDashboard.putNumber("Right Distance", getRightDistance());
    // System.out.println(getPitch());

  }

  public Pose2d getPos() {
    return  m_diffOdometry.getPoseMeters();

  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_diffOdometry.resetPosition(new Rotation2d(0), m_leftController1.getEncoder().getPosition(),
        m_rightController1.getEncoder().getPosition(), pose);
  }

  public double getYaw() {
    return m_navX.getYaw();
  }

  public double getPitch() {
    return m_navX.getPitch();
  }

  public double getRoll() {
    return m_navX.getRoll();
  }

  public double getVelocity() {
    return m_navX.getVelocityY();
  }

  public double getAngleK() {

    return m_navX.getAngle();
  }

  public void arcadeDrive(double velocity, double heading) {
    m_differentialdrive.arcadeDrive(velocity*m_throttle, -1 * heading*m_throttle);

    // System.out.println("velocity="+velocity);
    // System.out.println("heading="+heading);
    System.out.println("throttle="+m_throttle);
  }

  public void setMaxOutput(double maxOutput) {
    // m_drive.setMaxOutput(maxOutput);
    m_throttle= maxOutput;
  public void tankDriveVolts(double leftVolt, double rightVolt) {
    m_leftController1.setVoltage(leftVolt);
    m_rightController1.setVoltage(rightVolt);
    m_differentialdrive.feed();
  }

  private Rotation2d getRotation2dK() {
    // note the negation of the angle is required because the wpilib convention
    // uses left positive rotation while gyros read right positive
    return Rotation2d.fromDegrees(-getAngleK());
  }

  public void resetDrive() {
    resetAngle();
    resetEncoders();
  }

  private void resetAngle() {
    m_navX.zeroYaw();
    // need to add reset of odometry and encoders
  }

  private double getLeftDistance() {
    return m_leftEncoderSign * m_leftEncoder1.getPosition() * DriveConstants.kENCODER_DISTANCE_PER_PULSE_M;
  }

  private double getRightDistance() {
    return m_rightEncoder1.getPosition() * DriveConstants.kENCODER_DISTANCE_PER_PULSE_M;
    // return m_rightEncoderSign * m_rightEncoder1.getPosition() *
    // DriveConstants.kENCODER_DISTANCE_PER_PULSE_M;
  }

  private void resetEncoders() {
    m_leftEncoder1.setPosition(0);
    m_rightEncoder1.setPosition(0);
  }

}
