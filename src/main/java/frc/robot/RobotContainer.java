// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoControl;
import frc.robot.commands.AutoSelect;
import frc.robot.subsystems.ArmHandSubsystem;
import frc.robot.subsystems.ArmJointsSubsystem;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.libraries.CommandGamepadX;
import frc.robot.libraries.ConsoleAuto;
import frc.robot.libraries.GamepadX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.libraries.GamepadX;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * ON LOGITECH F310 CONTROLLER:
 * 
 * A = 1 (Green)
 * B = 2 (Red)
 * X = 3 (Blue)
 * Y = 4 (Yellow)
 * LB = 5 (Left Bumper: top button)
 * RB = 6 (Right-Bumper: top button)
 * Select/Back = 7 (Above left joystick)
 * Start = 8 (Above right joystick)
 * LJB = 9 (Press left joystick)
 * RJB = 10 (Press right joystick)
 * 
 * Left Joystick Vertical Axis = 1
 * Left Joystick Horizontal Axis = 0
 * Right Joystick Vertical Axis = 5
 * Right Joystick Horizontal Axis = 4
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmJointsSubsystem m_ArmJointsSubsystem = new ArmJointsSubsystem();

  private final ArmHandSubsystem m_ArmHand = new ArmHandSubsystem();
  private final ConsoleAuto m_consoleAuto = new ConsoleAuto(OperatorConstants.kAUTONOMOUS_CONSOLE_PORT);

  private final AutonomousSubsystem m_autonomous = new AutonomousSubsystem(m_consoleAuto,
    m_robotDrive,
    m_ArmJointsSubsystem,
   m_ArmHand
  );

  private final AutoSelect m_autoSelect = new AutoSelect(m_autonomous);
  private final AutoControl m_autoCommand = new AutoControl(m_autonomous, m_robotDrive, m_ArmHand, m_ArmJointsSubsystem);

  private final CommandGamepadX m_driverController = new CommandGamepadX(OperatorConstants.kDriverControllerPort);
  private final CommandGamepadX m_operatorController = new CommandGamepadX(OperatorConstants.kOPERATOR_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_robotDrive.setDefaultCommand(
        Commands.run(
            () -> m_robotDrive.arcadeDrive(m_driverController.getLeftY(), -m_driverController.getRightX()*Constants.DriveConstants.kTURN_SPEED),
            m_robotDrive));

    m_ArmJointsSubsystem.setDefaultCommand(
        Commands.run(
            () -> m_ArmJointsSubsystem.moveArm(m_operatorController.getLeftY(), m_operatorController.getRightY()),
            m_ArmJointsSubsystem));
    
    m_ArmHand.setDefaultCommand(
        Commands.run(
            () -> m_ArmHand.SetHandMotors(m_operatorController.getRightTrigger(),
                            m_operatorController.getLeftTigger()), 
                  m_ArmHand));
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_driverController
        .LB()
        .onTrue(Commands.runOnce(() -> m_robotDrive.setMaxOutput(Constants.DriveConstants.kLOW_GEAR_SPEED)))
        .onFalse(Commands.runOnce(() -> m_robotDrive.setMaxOutput(Constants.DriveConstants.kMEDIUM_GEAR_SPEED)));

    m_driverController
        .RB()
        .onTrue(Commands.runOnce(() -> m_robotDrive.setMaxOutput(Constants.DriveConstants.kHIGH_GEAR_SPEED)))
        .onFalse(Commands.runOnce(() -> m_robotDrive.setMaxOutput(Constants.DriveConstants.kMEDIUM_GEAR_SPEED)));

    m_operatorController
        .Start()
        .onTrue(Commands.runOnce(() -> m_ArmJointsSubsystem.stopArm(), m_ArmJointsSubsystem));
    m_operatorController
        .RB()
        .onTrue(Commands.runOnce(() -> m_ArmHand.OpenHand(), m_ArmHand));
    m_operatorController
        .LB()
        .onTrue(Commands.runOnce(() -> m_ArmHand.CloseHand(), m_ArmHand));
    /*m_operatorController
        .X()
        .onTrue(Commands.runOnce(() -> m_ArmHand.StopHandMotors(), m_ArmHand));
    */

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutoSelect() {
    return m_autoSelect;
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // Command autoCommand = ;
    return m_autoCommand;
  }
}
