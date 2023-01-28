// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitForCount extends CommandBase {
  /** Creates a new WaitForCount. */
  protected Timer m_timer = new Timer();
  private final double m_duration;
  IntSupplier m_isWaitCount;
  int m_iWaitCount;

  public WaitForCount(double seconds, IntSupplier waitCount) {
    m_isWaitCount = waitCount;
    m_duration = seconds;
    SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_iWaitCount = m_isWaitCount.getAsInt();
    m_iWaitCount--;
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isTimeUp = m_timer.hasElapsed(m_duration);
    if (m_iWaitCount < 0) {
      isTimeUp = true;
    }

    if (isTimeUp) {
      if (m_iWaitCount > 0) {
        isTimeUp = false;
        end(false);
        m_timer.reset();
        m_timer.start();
        m_iWaitCount--;
      }
    }
    return isTimeUp;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
