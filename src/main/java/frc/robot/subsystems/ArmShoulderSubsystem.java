// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmShoulderSubsystem extends SubsystemBase {
  /** Creates a new ArmShoulderSubsystem. */
  /*
   * single motor - motor controller tbd
   * limit switch on back and front 
   * motion limits - only move when elbow has forearm extended past bumper
   */
  public ArmShoulderSubsystem() {
    /*
     * define the controller, sensors, and defaults
     */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveShoulder(double speed) {
    // direct test of elbow sensor or depend on command? may be non-standard to direct test
    //set motor controller
  }

  public boolean isShoulderParked() {
    //retracted limit switch
    return true;
  }

  public boolean isShoulderExtended() {
    //forward limit switch
    return true;
  }
}
