// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmTurntable extends SubsystemBase {
  /** Creates a new ArmTurntable. */

  private WPI_TalonSRX m_TurnTableMotor;
  /*
   * single motor controller - most likely talon srx (CRTE libs)
   * limit switches - connect directly to motor controller
   * center - switch or sensor or ?
   *  switch simplest as a sensor but rotation position would have to be derived
   *  absolute encoder needs reference point
   *  potentiometer absolute and retains
   */
  public ArmTurntable() {
     m_TurnTableMotor = new WPI_TalonSRX (Constants.TurnTableConstants.kTURN_TABLE_MOTOR_PORT);
    /*
     * define the conroller/sensor stuff and set defaults
     */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isTurntableParked() {
    // center sensor
    return true;
  }

  public boolean isTurntableLeft() {
    // left limit switch
    return true;
  }

  public boolean isTurnableRight() {
    // right limit switch
    return true;
  }

  public void moveTurntable(double speed) {
    // set motor speed
    m_TurnTableMotor.set(speed);
  }




}
