// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  //private BuiltInAccelerometer mRioAccel;
  private int state;
  private int debounceCount;
  private double robotSpeedSlow;
  private double robotSpeedFast;
  private double onChargeStationDegree;
  private double levelDegree;
  private double debounceTime;
  //private double singleTapTime;
  //private double scoringBackUpTime;
  //private double doubleTapTime;
  private boolean bIsBalanced;
  private DriveSubsystem m_drive;

  public AutoBalance(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    //mRioAccel = new BuiltInAccelerometer();
    state = 0;
    debounceCount = 0;
    m_drive = drive;
    /**********
     * CONFIG *
     **********/
    // Speed the robot drived while scoring/approaching station, default = 0.4
    robotSpeedFast = 0.9;

    // Speed the robot drives while balancing itself on the charge station.
    // Should be roughly half the fast speed, to make the robot more accurate,
    // default = 0.2
    robotSpeedSlow = 0.2;

    // Angle where the robot knows it is on the charge station, default = 13.0
    onChargeStationDegree = 10.0;

    // Angle where the robot can assume it is level on the charging station
    // Used for exiting the drive forward sequence as well as for auto balancing,
    // default = 6.0
    levelDegree = 6.0;

    // Amount of time a sensor condition needs to be met before changing states in
    // seconds
    // Reduces the impact of sensor noice, but too high can make the auto run
    // slower, default = 0.2
    debounceTime = 0.5;

    // Amount of time to drive towards to scoring target when trying to bump the
    // game piece off
    // Time it takes to go from starting position to hit the scoring target
    //singleTapTime = 0.4;

    // Amount of time to drive away from knocked over gamepiece before the second
    // tap
    //scoringBackUpTime = 0.2;

    // Amount of time to drive forward to secure the scoring of the gamepiece
   // doubleTapTime = 0.3;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bIsBalanced = false;
    state = 0;
  }

  // returns the magnititude of the robot's tilt calculated by the root of
  // pitch^2 + roll^2, used to compensate for diagonally mounted rio
  public double getTilt() {
    double pitch = m_drive.getPitch();
    double roll = m_drive.getRoll();
    if ((pitch + roll) >= 0) {
      return Math.sqrt(pitch * pitch + roll * roll);
    } else {
      return -Math.sqrt(pitch * pitch + roll * roll);
    }
  }

  public int secondsToTicks(double time) {
    return (int) (time * 50);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(state);
    //System.out.println(getTilt());
    m_drive.arcadeDrive(-autoBalanceRoutine(), 0);
  }

  public double autoBalanceRoutine() {
    //state = 0;

    switch (state) {
      // drive forwards to approach station, exit when tilt is detected
      case 0:
        if (Math.abs(getTilt()) > onChargeStationDegree) {
          debounceCount++;
          //state = 1;
        }
        if (debounceCount > 20) {
          state = 1;
          debounceCount = 0;
          return robotSpeedFast;
        }
        return robotSpeedFast;
      // driving up charge station, drive slower, stopping when level
      case 1:
        if (getTilt() < levelDegree) {
          debounceCount++;
        }
        if (debounceCount > secondsToTicks(debounceTime)) {
          state = 2;
          debounceCount = 0;
          return 0;
        }
        return robotSpeedSlow;
      // on charge station, stop motors and wait for end of auto
      case 2:
        if (Math.abs(getTilt()) <= levelDegree / 2) {
          debounceCount++;
        }
        if (debounceCount > secondsToTicks(debounceTime)) {
          state = 3;
          debounceCount = 0;
          return 0;
        }
        if (getTilt() >= levelDegree) {
          return 0.1;
        } else if (getTilt() <= -levelDegree) {
          return -0.1;
        }
      case 3:
        bIsBalanced = true;
        return 0;
    }
    return 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bIsBalanced;
  }
}
