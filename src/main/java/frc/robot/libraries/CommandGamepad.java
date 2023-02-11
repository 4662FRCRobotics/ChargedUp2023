// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class CommandGamepad extends GenericHID{

    private final Gamepad m_hid;

    public CommandGamepad(int port) {
        super(port);
        m_hid = new Gamepad(port);
    }

    //@Override
    public Gamepad getHID() {
        return m_hid;
    }

    public double getLeftX() {
        return m_hid.getLeftX();
    }
    public double getLeftY() {
        return m_hid.getLeftY();
    }

    public double getRightX() {
        return m_hid.getRightX();
    }
    
    public double getRightY() {
        return m_hid.getRightY();
    }

    public Trigger LB() {
        return LB(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger LB(EventLoop loop) {
        return m_hid.LB(loop).castTo(Trigger::new);
    }

    public Trigger RB() {
        return RB(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger RB(EventLoop loop) {
        return m_hid.RB(loop).castTo(Trigger::new);
    }

    public Trigger X() {
        return X(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger X(EventLoop loop) {
        return m_hid.X(loop).castTo(Trigger::new);
    }
    public Trigger A() {
        return A(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger A(EventLoop loop) {
        return m_hid.A(loop).castTo(Trigger::new);
    }

    public Trigger B() {
        return B(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger B(EventLoop loop) {
        return m_hid.B(loop).castTo(Trigger::new);
    }
    public Trigger Y() {
        return Y(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger Y(EventLoop loop) {
        return m_hid.Y(loop).castTo(Trigger::new);
    }
    public Trigger Back() {
        return Back(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger Back(EventLoop loop) {
        return m_hid.Back(loop).castTo(Trigger::new);
    }

    public Trigger Start() {
        return Start(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger Start(EventLoop loop) {
        return m_hid.Start(loop).castTo(Trigger::new);
    }
    public Trigger LJB() {
        return LJB(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger LJB(EventLoop loop) {
        return m_hid.LJB(loop).castTo(Trigger::new);
    }
    public Trigger RJB() {
        return RJB(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger RJB(EventLoop loop) {
        return m_hid.RJB(loop).castTo(Trigger::new);
    }
}