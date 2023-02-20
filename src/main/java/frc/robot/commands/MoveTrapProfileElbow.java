// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmJointsSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveTrapProfileElbow extends TrapezoidProfileCommand {
  /** Creates a new MoveTrapProfileElbow. */
  public MoveTrapProfileElbow(double targetPosition, ArmJointsSubsystem armSubsystem) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
              ArmConstants.kELBOW_MAX_SPEED_PER_SECOND,
              ArmConstants.kELBOW_ACCELERATION_PER_SECOND_SQ),
            // Goal state
            new TrapezoidProfile.State(targetPosition, 0)),
        state -> armSubsystem.setElbowStates(state),
        armSubsystem
        );
  }
}
