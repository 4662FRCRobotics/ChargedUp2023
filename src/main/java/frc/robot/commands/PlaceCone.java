// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmHandSubsystem;

public class PlaceCone extends CommandBase {
  /** Creates a new PlaceCone. */
  ArmHandSubsystem m_armHand;
  public PlaceCone(ArmHandSubsystem armHand) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armHand = armHand;
    addRequirements(armHand);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // make sure arm is extended
    m_armHand.StartHandMotors( 1.0);
    m_armHand.OpenHand();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // call subsystem to run place
    //m_armHand.SetHandMotors(0.0, 1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armHand.StopHandMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // check that place is complete for true
    return false;
  }
}
