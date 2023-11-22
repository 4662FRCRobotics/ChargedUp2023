// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.libraries.ConsoleJoystick;

public class DriveSubsystem extends SubsystemBase {

  public enum DrivePath {
    SIMPLE (0, 0, -2.5, 0);

    private final double m_dStartX;
    private final double m_dStartY;
    private final double m_dMidX;
    private final double m_dMidY;
    private final double m_dEndX;
    private final double m_dEndY;

    private DrivePath(double dStartX, double dStartY, double dEndX, double dEndY)
    {
      this.m_dStartX = dStartX;
      this.m_dStartY = dStartY;
      this.m_dMidX = ((dStartX + dEndX) / 2);
      this.m_dMidY = ((dStartY + dEndY) / 2);
      this.m_dEndX = dEndX;
      this.m_dEndY = dEndY;
    }
    
    private DrivePath(double dStartX, double dStartY, double dMidX, double dMidY, double dEndX, double dEndY)
    {
      this.m_dStartX = dStartX;
      this.m_dStartY = dStartY;
      this.m_dMidX = dMidX;
      this.m_dMidY = dMidY;
      this.m_dEndX = dEndX;
      this.m_dEndY = dEndY;
    }

    double getStartX(){
      return m_dStartX;
    }

    double getStartY(){
      return m_dStartY;
    }

    double getMidX() {
      return m_dMidX;
    }

    double getMidY(){
      return m_dMidY;
    }

    double getEndX(){
      return m_dEndX;
    }

    double getEndY(){
      return m_dEndY;
    }

  }
  /** Creates a new Drive. */
  private ConsoleJoystick m_console;
  private CANSparkMax m_leftController1;
  private CANSparkMax m_leftController2;
  private CANSparkMax m_rightController1;
  private CANSparkMax m_rightController2;
  private DifferentialDrive m_differentialdrive;
  private double m_throttle;
  private AHRS m_navX;
  private SimpleMotorFeedforward m_feedForward;
  private DifferentialDriveVoltageConstraint m_autoVoltageConstraint;
  private TrajectoryConfig m_trajectoryConfig;
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

    m_feedForward = new SimpleMotorFeedforward(
      DriveConstants.kS_VOLTS,
      DriveConstants.kV_VOLT_SECOND_PER_METER,
      DriveConstants.kA_VOLT_SEONDS_SQUARED_PER_METER);

  //  System.out.printf("Encoder dist per pulse %.4f", DriveNormSubsystemConst.kENCODER_DISTANCE_PER_PULSE_M);

    m_autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      m_feedForward,
      DriveConstants.kDRIVE_KINEMATICS,
      DriveConstants.kMAX_VOLTAGE);
      
    m_trajectoryConfig = new TrajectoryConfig(
      DriveConstants.kMAX_SPEED_METERS_PER_SECOND,
      DriveConstants.kMAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
      .setKinematics(DriveConstants.kDRIVE_KINEMATICS)
      .addConstraint(m_autoVoltageConstraint);
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

  public TrajectoryConfig getTrajConfig() {
    return m_trajectoryConfig;
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
    //System.out.println("throttle="+m_throttle);
  }

  public void setMaxOutput(double maxOutput) {
    // m_drive.setMaxOutput(maxOutput);
    m_throttle = maxOutput;
  }

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
    // that was done
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

  // generate an internal trajectory using specified begin, way points, and end

  private Trajectory genTrajectory(DrivePath drivePath) {
    return TrajectoryGenerator.generateTrajectory(
        new Pose2d(drivePath.getStartX(), drivePath.getStartY(), new Rotation2d(0)),
        List.of(new Translation2d(drivePath.getMidX(), drivePath.getMidY())),
        new Pose2d(drivePath.getEndX(), drivePath.getEndY(), new Rotation2d(0)),
        getTrajConfig());
  }
  // change posistions to constants at home

  // read externally generated trajectory (path) from an external file in the
  // standard "deploy" path
  // these are generated from a standard tool such as pathweaver
  private Trajectory readPaths(String jsonPath) {
    Trajectory trajectory = null;
    try {
      Path trajPath = Filesystem.getDeployDirectory().toPath().resolve(jsonPath);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajPath);
    } catch (IOException ex) {

    }
    return trajectory;
  }

  public Command getRamsetePath(DrivePath drivePath) {
   
    // use the voltage constraints and etc. from the constructor method

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = genTrajectory(drivePath);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            () -> getPos(),
            new RamseteController(),
            //new SimpleMotorFeedforward(
            //    DriveConstants.ksVolts,
            //    DriveConstants.kvVoltSecondsPerMeter,
            //    DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDRIVE_KINEMATICS,
            //this::getWheelSpeeds,
            //new PIDController(DriveConstants.kPDriveVel, 0, 0),
            //new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            this::tankDriveVolts,
            this);

    // Reset odometry to the starting pose of the trajectory.
    resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> tankDriveVolts(0, 0));
  }

}
