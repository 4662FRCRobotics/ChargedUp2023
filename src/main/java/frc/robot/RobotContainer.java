// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoControl;
import frc.robot.commands.AutoSelect;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.libraries.ConsoleAuto;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.libraries.Gamepad;
import frc.robot.subsystems.DriveSubsytem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.libraries.Gamepad;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
ON LOGITECH F310 CONTROLLER:
X = 1  (Blue)
a = 2  (Green)
B = 3   (Red)
Y = 4   (Yellow)
LB = 5 (Left Bumper: top button)
RB = 6  (Right-Bumper: top button)
LT = 7  (Left-Trigger: bottom button)
RT = 8  (Right-Trigger: bottom button)
Select/Back = 9 (Above left joystick) 
Start = 10 (Above right joystick)
LJB = 11 (Press left joystick)
RJB = 12   (Press right joystick)

Left Joystick Vertical Axis = 1
Left Joystick Horizontal Axis = 0
Right Joystick Vertical Axis = 3
Right Joystick Horizontal Axis = 2 
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final ConsoleAuto m_consoleAuto = new ConsoleAuto(OperatorConstants.kAUTONOMOUS_CONSOLE_PORT);

  private final AutonomousSubsystem m_autonomous = new AutonomousSubsystem(m_consoleAuto, m_robotDrive);

  private final AutoSelect m_autoSelect = new AutoSelect(m_autonomous);
  private final AutoControl m_autoCommand = new AutoControl(m_autonomous, m_robotDrive);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Gamepad m_driverController = new Gamepad(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_robotDrive.setDefaultCommand(
      Commands.run(
        () ->
        m_robotDrive.arcadeDrive(m_driverController.getLeftY()*(1-((m_driverController.getRightY()+1)*.25)), -m_driverController.getRightX()*(1-((m_driverController.getRightY()+1)*.25))),
         m_robotDrive)
        );
    //m_drive.arcadeDrive(m_driverController.getLeftY()*(1-((m_driverController.getRightY()+1)*.25)), -m_driverController.getRightX()*(1-((m_driverController.getRightY()+1)*.25))),
    //     m_drive)
            //m_drive.arcadeDrive((m_driverController.getY()*-1*(1-((m_driverController.getThrottle()+1)*.25))), -m_driverController.getZ()), m_drive)
    // Configure the trigger bindings
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutoSelect(){
    return m_autoSelect;
  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    Command autoCommand = null; 
    return autoCommand;
  }
}
