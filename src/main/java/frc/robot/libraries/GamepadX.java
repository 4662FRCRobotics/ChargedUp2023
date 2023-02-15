// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries;

import java.util.function.BooleanSupplier;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GamepadX extends GenericHID {
  /** Creates a new Gamepad. */
  public static final byte kLeftHorizontalAxis = 0;
  public static final byte kLeftVerticalAxis = 1;
  public static final byte kLeftTriggerAxis = 2;
  public static final byte kRightTriggerAxis = 3;
  public static final byte kRightHorizontalAxis = 4;
  public static final byte kRightVerticalAxis = 5;

  public enum AxisType {
    kLeftX(0),
    kLeftY(1),
    kLeftTrigger(2),
    KRightTrigger(3),
    kRightX(4),
    KRightY(5);

    public final int value;

    AxisType(int value) {
      this.value = value;
    }
  }

  public enum ROTSW {
    kDpad(0);

    public final int value;

    ROTSW(int value) {
      this.value = value;
    }
  }

  public enum ButtonType {
    kX(3),
    kA(1),
    kB(2),
    kY(4),
    kLB(5),
    kRB(6),
   // kLT(7),
    //kRT(8),
    kBack(7),
    kStart(8),
    kLJB(9),
    kRJB(10);

    public final int value;

    ButtonType(int value) {
      this.value = value;
    }
  }

  private final byte[] m_axes = new byte[AxisType.values().length];

  public GamepadX(final int port) {
    super(port);
    //HAL.report(tResourceType.kResourceType_l, port + 1); no gamepad

    m_axes[AxisType.kLeftX.value] = kLeftHorizontalAxis;
    m_axes[AxisType.kLeftY.value] = kLeftVerticalAxis;
    m_axes[AxisType.kRightX.value] = kRightHorizontalAxis;
    m_axes[AxisType.KRightY.value] = kRightVerticalAxis;
    m_axes[AxisType.kLeftTrigger.value] = kLeftTriggerAxis;
    m_axes[AxisType.KRightTrigger.value] = kRightTriggerAxis;
  }

  public final double getLeftX() {
    return getRawAxis(m_axes[AxisType.kLeftX.value]);
  }

  public final double getLeftY() {
    return getRawAxis(m_axes[AxisType.kLeftY.value]);
  }

  public final double getLeftTrigger() {
    return getRawAxis(m_axes[AxisType.kLeftTrigger.value]);
  }
  public final double getRightTrigger() {
    return getRawAxis(m_axes[AxisType.KRightTrigger.value]);
  }
  public final double getRightX() {
    return getRawAxis(m_axes[AxisType.kRightX.value]);
  }

  public final double getRightY() {
    return getRawAxis(m_axes[AxisType.KRightY.value]);
  }

  //public int getPOV() {
   // return getRawAxis(ROTSW.kDpad.value);
 // }

  public boolean getX() {
    return getRawButton(ButtonType.kX.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent X (EventLoop loop) {
    return new BooleanEvent(loop, () -> getX());
  }

  public boolean getA() {
    return getRawButton(ButtonType.kA.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent A (EventLoop loop) {
    return new BooleanEvent(loop, () -> getA());
  }

  public boolean getB() {
    return getRawButton(ButtonType.kB.value);
  }
  @SuppressWarnings("MethodName")
  public BooleanEvent B(EventLoop loop) {
    return new BooleanEvent(loop, () -> getB());
  }
  public boolean getY() {
    return getRawButton(ButtonType.kY.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent Y(EventLoop loop) {
    return new BooleanEvent(loop, () -> getY());
  }

  public boolean getLB() {
    return getRawButton(ButtonType.kLB.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent LB(EventLoop loop) {
    return new BooleanEvent(loop, () -> getLB());
  }

  public boolean getRB() {
    return getRawButton(ButtonType.kRB.value);
  }

  
  @SuppressWarnings("MethodName")
  public BooleanEvent RB(EventLoop loop) {
    return new BooleanEvent(loop, () -> getRB());
  }

 /*  public boolean getLT() {
    return getRawButton(ButtonType.kLT.value);
  }

  public boolean getRT() {
    return getRawButton(ButtonType.kRT.value);
  }
*/
  public boolean getBack() {
    return getRawButton(ButtonType.kBack.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent Back (EventLoop loop) {
    return new BooleanEvent(loop, () -> getBack());
  }
  public boolean getStart() {
    return getRawButton(ButtonType.kX.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent Start (EventLoop loop) {
    return new BooleanEvent(loop, () -> getStart());
  }

  public boolean getLJB() {
    return getRawButton(ButtonType.kLJB.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent LJB(EventLoop loop) {
    return new BooleanEvent(loop, () -> getLJB());
  }

  public boolean getRJB() {
    return getRawButton(ButtonType.kRJB.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent RJB(EventLoop loop) {
    return new BooleanEvent(loop, () -> getRJB());
  }

}
