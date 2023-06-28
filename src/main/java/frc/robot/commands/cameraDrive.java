// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

public class cameraDrive extends CommandBase {
  /** Creates a new cameraDrive. */
  private DriveSubsystem m_drive;
  private Vision m_Vision;
  public cameraDrive(DriveSubsystem drive, Vision vision) {
    m_drive = drive;
m_Vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive,m_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(0, m_Vision.GetTurn());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_Vision.haveTarget();
  }
}
