// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
<<<<<<< HEAD

import frc.robot.subsystems.DriveSubsystem;

public class RamseteDrivePath extends RamseteCommand {
  /** Creates a new DriveRamsetePath. */
  private DriveSubsystem m_drive;
  private Trajectory m_trajectory;
  private boolean m_resetOdometry;

  public RamseteDrivePath(Trajectory trajectory, boolean resetOdometry, DriveSubsystem drive) {
=======
import frc.robot.subsystems.Drive;

public class RamseteDrivePath extends RamseteCommand {
  /** Creates a new DriveRamsetePath. */
  private Drive m_drive;
  private Trajectory m_trajectory;
  private boolean m_resetOdometry;

  public RamseteDrivePath(Trajectory trajectory, boolean resetOdometry, Drive drive) {
>>>>>>> 7f94994eaa96f472ea76fee7f58591d550693112
    // Use addRequirements() here to declare subsystem dependencies.
    // alternate construction omits feedfwd, wheelspeeds, and pidcontrollers
    // two outputs - the simple (non PIDController) sends wheel speeds, not volts!!!
    // This needs testing on the ground somehow.
    // PIDController needs serious tuning and testing
    super(trajectory,
        drive::getPos,
        new RamseteController(),
        // drive.getSmpFeedFwd(),
        Constants.DriveConstants.kDRIVE_KINEMATICS,
        // drive::getWheelSpeeds,
        // new PIDController(DriveNormSubsystemConst.kRAMSETE_P, 0,
        // DriveNormSubsystemConst.kRAMSETE_D),
        // new PIDController(DriveNormSubsystemConst.kRAMSETE_P, 0,
        // DriveNormSubsystemConst.kRAMSETE_D),
        drive::tankDriveVolts,
        drive);
    m_trajectory = trajectory;
    m_resetOdometry = resetOdometry;
    m_drive = drive;
  }

<<<<<<< HEAD
  public RamseteDrivePath(Trajectory trajectory, DriveSubsystem drive) {
=======
  public RamseteDrivePath(Trajectory trajectory, Drive drive) {
>>>>>>> 7f94994eaa96f472ea76fee7f58591d550693112
    this(trajectory, false, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    if (m_resetOdometry) {
      m_drive.resetOdometry(m_trajectory.getInitialPose());
    }
  }
  /*
   * use super inherited methods
   * // Called every time the scheduler runs while the command is scheduled.
   * 
   * @Override
   * public void execute() {}
   * // Called once the command ends or is interrupted.
   * 
   * @Override
   * public void end(boolean interrupted) {}
   * // Returns true when the command should end.
   * 
   * @Override
   * public boolean isFinished() {
   * return false;
   * }
   */
}