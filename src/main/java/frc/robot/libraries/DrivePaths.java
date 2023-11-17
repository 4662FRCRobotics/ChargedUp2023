// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public enum DrivePaths {
    
PATH0( new Pose2d(0, 0, new Rotation2d(0)),
List.of(new Translation2d(-1.5, 0)),
new Pose2d(-2.5, 0, new Rotation2d(0)),
m_drive.getTrajConfig()),
PATH1,
PATH2;

private DriveSubsystem m_drive;
}
