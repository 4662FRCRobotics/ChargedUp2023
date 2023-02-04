// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class Gamepad extends GenericHID {
  /** Creates a new Gamepad. */
  public static final byte kLeftHorizontalAxis = 0;
  public static final byte kLeftVerticalAxis = 1;
  public static final byte kRightHorizontalAxis = 4;
  public static final byte kRightVerticalAxis = 5;

  public enum AxisType {
    kLeftX(0),
    kLeftY(1),
    kRightX(2),
    KRightY(3);

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
    kX(1),
    kA(2),
    kB(3),
    kY(4),
    kLB(5),
    kRB(6),
    kLT(7),
    kRT(8),
    kBack(9),
    kStart(10),
    kLJB(11),
    kRJB(12);

    public final int value;

    ButtonType(int value) {
      this.value = value;
    }
  }

  private final byte[] m_axes = new byte[AxisType.values().length];

  public Gamepad(final int port) {
    super(port);

    m_axes[AxisType.kLeftX.value] = kLeftHorizontalAxis;
    m_axes[AxisType.kLeftY.value] = kLeftVerticalAxis;
    m_axes[AxisType.kRightX.value] = kRightHorizontalAxis;
    m_axes[AxisType.KRightY.value] = kRightVerticalAxis;
  }

  public final double getLeftX() {
    return getRawAxis(m_axes[AxisType.kLeftX.value]);
  }

  public final double getLeftY() {
    return getRawAxis(m_axes[AxisType.kLeftY.value]);
  }

  public final double getRightX() {
    return getRawAxis(m_axes[AxisType.kRightX.value]);
  }

  public final double getRightY() {
    return getRawAxis(m_axes[AxisType.KRightY.value]);
  }

  /*public int getPOV() {
    return getRawButton(ROTSW.kDpad.value);
  }*/

  public boolean getX() {
    return getRawButton(ButtonType.kX.value);
  }

  public boolean getA() {
    return getRawButton(ButtonType.kA.value);
  }

  public boolean getB() {
    return getRawButton(ButtonType.kB.value);
  }

  public boolean getY() {
    return getRawButton(ButtonType.kY.value);
  }

  public boolean getLB() {
    return getRawButton(ButtonType.kLB.value);
  }

  public boolean getRB() {
    return getRawButton(ButtonType.kRB.value);
  }

  public boolean getLT() {
    return getRawButton(ButtonType.kLT.value);
  }

  public boolean getRT() {
    return getRawButton(ButtonType.kRT.value);
  }

  public boolean getBack() {
    return getRawButton(ButtonType.kBack.value);
  }

  public boolean getStart() {
    return getRawButton(ButtonType.kX.value);
  }

  public boolean getLJB() {
    return getRawButton(ButtonType.kLJB.value);
  }

  public boolean getRJB() {
    return getRawButton(ButtonType.kRJB.value);
  }
}
