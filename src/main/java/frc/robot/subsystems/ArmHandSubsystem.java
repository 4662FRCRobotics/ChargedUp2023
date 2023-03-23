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
import frc.robot.Constants.HandConstants;
import frc.robot.Constants.PneumaticsConstants;;

public class ArmHandSubsystem extends SubsystemBase {
  /** Creates a new ArmHand. */
  private CANSparkMax LeftHandMotor;
  private CANSparkMax RightHandMotor;
  private DoubleSolenoid HandCloser;

  public ArmHandSubsystem() {
    LeftHandMotor = new CANSparkMax(HandConstants.kLEFTMOTORPORT, MotorType.kBrushless);
    RightHandMotor = new CANSparkMax(HandConstants.kRIGHTMOTORPORT, MotorType.kBrushless);

    LeftHandMotor.restoreFactoryDefaults();
    RightHandMotor.restoreFactoryDefaults();

    LeftHandMotor.setIdleMode(IdleMode.kBrake);
    RightHandMotor.setIdleMode(IdleMode.kBrake);

    LeftHandMotor.setInverted(!HandConstants.kIS_INVERTED);
    RightHandMotor.setInverted(HandConstants.kIS_INVERTED);

    LeftHandMotor.setSmartCurrentLimit(HandConstants.kMAX_HAND_AMPS);
    RightHandMotor.setSmartCurrentLimit(HandConstants.kMAX_HAND_AMPS);

    LeftHandMotor.setOpenLoopRampRate(HandConstants.kHAND_RAMP_RATE);
    RightHandMotor.setOpenLoopRampRate(HandConstants.kHAND_RAMP_RATE);
    
    HandCloser = new DoubleSolenoid(PneumaticsConstants.kPNEUMATIC_HUB_PORT, PneumaticsModuleType.REVPH,
        HandConstants.kFWDPORT, HandConstants.kREVPORT);
  }

  public void CloseHand() {
    HandCloser.set(Value.kForward);
  }

  public void OpenHand() {
    HandCloser.set(Value.kReverse);
  }

  public void SetHandMotors(Double inSpeed, double outSpeed) {
    double speed = outSpeed - inSpeed;
    LeftHandMotor.set(speed * HandConstants.kSPEED_FACTOR);
    RightHandMotor.set(speed * HandConstants.kSPEED_FACTOR);
  }

  public void StopHandMotors() {
    LeftHandMotor.stopMotor();
    RightHandMotor.stopMotor();
  }

  public void StartHandMotors(double speed){
    LeftHandMotor.set(speed);
    RightHandMotor.set(speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
