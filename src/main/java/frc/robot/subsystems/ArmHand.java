// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmHand extends SubsystemBase {
  /** Creates a new ArmHand. */
  private CANSparkMax LeftHandMotor;
  private CANSparkMax RightHandMotor;
  private DoubleSolenoid HandCloser;

  public ArmHand() {
    LeftHandMotor = new CANSparkMax(Constants.HandConstants.kLEFTMOTORPORT, MotorType.kBrushless);
    RightHandMotor = new CANSparkMax(Constants.HandConstants.kRIGHTMOTORPORT, MotorType.kBrushless);

    LeftHandMotor.restoreFactoryDefaults();
    RightHandMotor.restoreFactoryDefaults();

    LeftHandMotor.setIdleMode(IdleMode.kBrake);
    RightHandMotor.setIdleMode(IdleMode.kBrake);

    LeftHandMotor.setInverted(!Constants.HandConstants.kIS_INVERTED);
    RightHandMotor.setInverted(Constants.HandConstants.kIS_INVERTED);

    LeftHandMotor.setSmartCurrentLimit(Constants.HandConstants.kMAX_HAND_AMPS);
    RightHandMotor.setSmartCurrentLimit(Constants.HandConstants.kMAX_HAND_AMPS);

    LeftHandMotor.setOpenLoopRampRate(Constants.HandConstants.kHAND_RAMP_RATE);
    RightHandMotor.setOpenLoopRampRate(Constants.HandConstants.kHAND_RAMP_RATE);
    
    HandCloser = new DoubleSolenoid(Constants.PneumaticsConstants.kPNEUMATIC_HUB_PORT, PneumaticsModuleType.REVPH,
        Constants.HandConstants.FWDPORT, Constants.HandConstants.REVPORT);
  }

  public void CloseHand() {
    HandCloser.set(Value.kForward);
  }

  public void OpenHand() {
    HandCloser.set(Value.kReverse);
  }

  public void SetHandMotors(Double Speed) {
    LeftHandMotor.set(Speed);
    RightHandMotor.set(Speed);
  }

  public void StopHandMotors() {
    LeftHandMotor.stopMotor();
    RightHandMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
