// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

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
  private CANSparkMax m_elbowMotor;

  private SparkMaxLimitSwitch m_elbowRevLimit;
  private SparkMaxLimitSwitch m_elbowFwdLimit;
  private SparkMaxAbsoluteEncoder m_elbowAngle;
  private SparkMaxPIDController m_elbowPIDCntl;
  private SimpleMotorFeedforward m_elbowFeedforward;

  public ArmJointsSubsystem() {
    m_ShoulderMotor = new WPI_TalonSRX(Constants.ArmConstants.kShoulderPort);
    m_elbowMotor = new CANSparkMax(Constants.ArmConstants.kELBOW_PORT, MotorType.kBrushless);

    m_ShoulderMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen, Constants.ArmConstants.kShoulderPort);
    m_ShoulderMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen, Constants.ArmConstants.kShoulderPort);

    m_elbowRevLimit = m_elbowMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_elbowFwdLimit = m_elbowMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_elbowFwdLimit.enableLimitSwitch(true);
    m_elbowRevLimit.enableLimitSwitch(true);

    //m_elbowAngle = new DutyCycleEncoder(Constants.ArmConstants.kELBOW_ANGLE_PORT);
    m_elbowAngle = m_elbowMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    m_elbowAngle.setPositionConversionFactor(ArmConstants.kELBOW_ANGLE_CONVERSION);
    m_elbowPIDCntl = m_elbowMotor.getPIDController();
    m_elbowPIDCntl.setFeedbackDevice(m_elbowAngle);
    m_elbowPIDCntl.setP(ArmConstants.kELBOW_CNTL_P);
    m_elbowPIDCntl.setI(ArmConstants.kELBOW_CNTL_I);
    m_elbowPIDCntl.setD(ArmConstants.kELBOW_CNTL_D);
    m_elbowPIDCntl.setIZone(ArmConstants.kELBOW_CNTL_IZONE);
    m_elbowPIDCntl.setOutputRange(ArmConstants.kELBOW_CNTL_MIN_OUT, ArmConstants.kELBOW_CNTL_MAX_OUT);
    m_elbowFeedforward = new SimpleMotorFeedforward(ArmConstants.kELBOW_FF_S_VOLTS,
      ArmConstants.kELBOW_FF_V_VOLT_SECOND_PER_UNIT,
      ArmConstants.kELBOW_FF_A_VOLT_SECOND_SQ_PER_UNIT);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("angle of elbow encoder", m_elbowAngle.getPosition());
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
      if (m_elbowAngle.getPosition() > Constants.ArmConstants.kBUMPER_SETPOINT) {
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

    boolean isElbowMoveFwd = true;

    if (speed > 0) {
      isElbowMoveFwd = false;
    }
    if (canElbowMove(isElbowMoveFwd)) {
      // set speed here
    } else {
      // stop elbow motor
    }

    if (speed < 0) {
      //arm is moving back
      if (isShoulderParked()) {
        m_elbowMotor.set(speed);
      } else {
        if (m_elbowAngle.getPosition() > Constants.ArmConstants.kBUMPER_SETPOINT) {
          m_elbowMotor.set(speed);
        } else {
          
          m_elbowMotor.set(0);
        }
      }
    }else{
      //arm is moving out
      if (m_elbowAngle.getPosition() < Constants.ArmConstants.kELBOW_TOP_LIMIT){
        m_elbowMotor.set(speed);
      }
    }

    m_elbowMotor.set(speed);
  }

  public void setElbowStates(TrapezoidProfile.State elbowState) {
    // determine move direction
    boolean isElbowMoveFwd = true;
    if (canElbowMove(isElbowMoveFwd)) {
      m_elbowPIDCntl.setReference(elbowState.position,
        ControlType.kPosition,
        0,
        m_elbowFeedforward.calculate(elbowState.velocity));
    }
  }

  public boolean canElbowMove(boolean isElbowMoveFwd) {
    boolean isElbowMovable = true;
    return isElbowMovable;
  }

  public void stopElbowMove() {
    m_elbowMotor.stopMotor();
  }

}
