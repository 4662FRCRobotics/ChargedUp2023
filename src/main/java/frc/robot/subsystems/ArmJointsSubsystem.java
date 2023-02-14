// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmJointsSubsystem extends SubsystemBase {
  /** Creates a new ArmShoulderSubsystem. */
  /*
   * single motor - motor controller tbd
   * limit switch on back and front
   * motion limits - only move when elbow has forearm extended past bumper
   */
  private boolean m_isArmParked;
  private boolean m_isArmExtended;
  private WPI_TalonSRX m_ShoulderMotor;
  private CANSparkMax m_elbow;

  private SparkMaxLimitSwitch m_elbowRevLimit;
  private SparkMaxLimitSwitch m_elbowFwdLimit;
  private DutyCycleEncoder m_elbowAngle;

  public ArmJointsSubsystem() {
    m_ShoulderMotor = new WPI_TalonSRX(Constants.ArmConstants.kShoulderPort);
    m_elbow = new CANSparkMax(Constants.ArmConstants.kELBOW_PORT, MotorType.kBrushless);

    m_ShoulderMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen, Constants.ArmConstants.kShoulderPort);
    m_ShoulderMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen, Constants.ArmConstants.kShoulderPort);

    m_elbowRevLimit = m_elbow.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_elbowFwdLimit = m_elbow.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_elbowFwdLimit.enableLimitSwitch(true);
    m_elbowRevLimit.enableLimitSwitch(true);

    m_elbowAngle = new DutyCycleEncoder(Constants.ArmConstants.kELBOW_ANGLE_PORT);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("angle of elbow encoder", m_elbowAngle.getAbsolutePosition());
    // This method will be called once per scheduler run
  }

  public void moveArm(double speed) {
    moveShoulder(speed);
    moveElbow(speed);
  }

  public void moveShoulder(double speed) {
    // direct test of elbow sensor or depend on command? may be non-standard to
    // direct test
    // set motor controller
    if (speed > 0) {
      if (m_elbowAngle.getAbsolutePosition() > Constants.ArmConstants.kBUMPER_SETPOINT) {
        m_ShoulderMotor.set(speed);
      }
    } else {
      m_ShoulderMotor.set(speed);
    }

  }

  public boolean isShoulderParked() {
    // retracted limit switch
    m_isArmParked = m_ShoulderMotor.isRevLimitSwitchClosed() == 1;
    return m_isArmParked;
  }

  public boolean isShoulderExtended() {
    // forward limit switch
    m_isArmExtended = m_ShoulderMotor.isFwdLimitSwitchClosed() == 1;
    return m_isArmExtended;
  }
/*
 *speed <0 is back  
 *elbow angle is an asanding num 
 */
  public void moveElbow(double speed) {

    if (speed < 0) {
      //arm is moving back
      if (isShoulderParked()) {
        m_elbow.set(speed);
      } else {
        if (m_elbowAngle.getAbsolutePosition() > Constants.ArmConstants.kBUMPER_SETPOINT) {
          m_elbow.set(speed);
        } else {
          
          m_elbow.set(0);
        }
      }
    }else{
      //arm is moving out
      if (m_elbowAngle.getAbsolutePosition() < Constants.ArmConstants.kELBOW_TOP_LIMIT){
        m_elbow.set(speed);
      }
    }

    m_elbow.set(speed);
  }

}
