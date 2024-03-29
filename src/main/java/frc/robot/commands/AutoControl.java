package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmHandSubsystem;
import frc.robot.subsystems.ArmJointsSubsystem;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutoControl extends CommandBase {

    private Command m_currentCommand;
    private AutonomousSubsystem m_autonomous;
    private DriveSubsystem m_drive;
    private ArmHandSubsystem m_hand;
    private ArmJointsSubsystem m_ArmJointsSubsystem;

    public AutoControl(AutonomousSubsystem autonomous, DriveSubsystem drive, ArmHandSubsystem hand, ArmJointsSubsystem armjoints) {
        m_autonomous = autonomous;
        m_drive = drive;
        m_hand = hand;
        m_ArmJointsSubsystem = armjoints;
        addRequirements(m_autonomous, m_drive, m_hand, armjoints);
    }

    @Override
    public void initialize() {
        // getInstance();
        m_autonomous.initGetCommand();
        m_currentCommand = m_autonomous.getNextCommand();
        m_currentCommand.initialize();
    }

    @Override
    public void execute() {
        m_currentCommand.execute();
        // System.out.println("execute");
    }

    @Override
    public void end(boolean interrupted) {
        m_currentCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        boolean areWeThereYet = true;
        Command nextCommand = null;
        if (m_currentCommand.isFinished() == false) {
            areWeThereYet = false;
        } else {
            nextCommand = m_autonomous.getNextCommand();
            if (nextCommand != null) {
                switchCommand(nextCommand);
                areWeThereYet = false;
            }
        }
        return areWeThereYet;
    }

    // stops current command then goes to next one
    private void switchCommand(final Command cmd) {
        m_currentCommand.end(false);
        m_currentCommand = cmd;
        m_currentCommand.initialize();
    }

}