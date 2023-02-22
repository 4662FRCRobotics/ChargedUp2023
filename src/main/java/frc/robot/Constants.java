// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class PneumaticsConstants {
    public static final int kPNEUMATIC_HUB_PORT = 1;
  }

  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 2;
    public static final int kLeftMotor2Port = 3;
    public static final int kRightMotor1Port = 4;
    public static final int kRightMotor2Port = 5;
    public static final boolean kIS_DRIVE_INVERTED = false;

    public static final double kRAMP_RATE = 1.0;
    public static final int kCURRENT_LIMT = 40;
    public static final double kMAX_VOLTAGE = 10;

    public static final double kTIRE_SIZE_IN = 6.0;
    public static final double kTIRE_SIZE_M = Units.inchesToMeters(kTIRE_SIZE_IN);
    public static final int kPULSE_PER_ROTATION = 1;

    public static final double kGEAR_REDUCTION = (52.0 / 12.0) * (68.0 / 30.0);
    // public static final double kHIGH_GEAR_REDUCTION = (42.0 / 12.0) * (50.0 /
    // 24.0);
    // public static final double kENCODER_DISTANCE_PER_PULSE_M_HIGH = ((double)
    // kPULSE_PER_ROTATION / kHIGH_GEAR_REDUCTION);
    public static final double kENCODER_DISTANCE_PER_PULSE_M = ((double) kPULSE_PER_ROTATION / kGEAR_REDUCTION);
    // * (kTIRE_SIZE_M * Math.PI);
    public static final double kTRACK_WIDTH_M = 0.5;

    // public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS = new
    // DifferentialDriveKinematics(kTRACK_WIDTH_M);

    public static final double kS_VOLTS = 0.0734;
    public static final double kV_VOLT_SECOND_PER_METER = 2.7;
    public static final double kA_VOLT_SEONDS_SQUARED_PER_METER = 0.074;
    public static final double kMAX_SPEED_METERS_PER_SECOND = 4;
    public static final double kMAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;
    public static final double kLOW_GEAR_SPEED = .5;
    public static final double kMEDIUM_GEAR_SPEED = .75;
    public static final double kHIGH_GEAR_SPEED = 1;

    public static final DifferentialDriveKinematics kDRIVE_KINEMATICS = new DifferentialDriveKinematics(kTRACK_WIDTH_M);
    // shifter numatic ports
    public static final int kSHIFT_UP = 1;
    public static final int kSHIFT_DOWN = 0;

    public static final double kDRIVE_P = 0.3;
    public static final double kDRIVE_I = 0.0;
    public static final double kDRIVE_D = 0.0;
    public static final double kDRIVE_TOLERANCE = 2;
    public static final double kDRIVE_PID_LIMIT = 0.75;
    public static final double kTURN_PID_LIMIT = 0.6;
    public static final double kTURN_ANGLE_P = 0.33;
    public static final double kTURN_ANGLE_I = 0.0;
    public static final double kTURN_ANGLE_D = 0.41;
    public static final double kTURN_ANGLE_TOLERANCE = 1;
    public static final double kTURN_ANGLE_TOLERANCE_DEG_PER_S = 10;
    /*
     * public static final double kKEEP_HEADING_P = 0.2;
     * public static final double kKEEP_HEADING_I = 0.0;
     * public static final double kKEEP_HEADING_D = 0.4;
     * public static final double kKEEP_HEADING_TOLERANCE = 1;
     */
  }

  public static class ArmConstants {
    public static final int kShoulderPort = 12;
    public static final int kELBOW_PORT = 11;

    // convert abs encoder angle to degrees just because
    public static final double kELBOW_ANGLE_CONVERSION = 360;
    public static final double kBUMPER_SETPOINT = 108;
    public static final double kELBOW_TOP_LIMIT = 252;

    public static final double kELBOW_CNTL_P = .15;
    public static final double kELBOW_CNTL_I = 0;
    public static final double kELBOW_CNTL_D = 3;
    public static final double kELBOW_CNTL_IZONE = 0;
    public static final double kELBOW_CNTL_FF = 0;
    public static final double kELBOW_CNTL_MAX_OUT = 1.0;
    public static final double kELBOW_CNTL_MIN_OUT = -1.0;

    public static final double kELBOW_FF_S_VOLTS = 0.5;
    public static final double kELBOW_FF_V_VOLT_SECOND_PER_UNIT = 1;
    public static final double kELBOW_FF_A_VOLT_SECOND_SQ_PER_UNIT = .05;

    public static final double kELBOW_MAX_SPEED_PER_SECOND = 0.5;
    public static final double kELBOW_ACCELERATION_PER_SECOND_SQ = 0.25;

  }

  public static class HandConstants {
    public static final int kLEFTMOTORPORT = 20;
    public static final int kRIGHTMOTORPORT = 21;
    public static final boolean kIS_INVERTED = true;
    public static final int FWDPORT = 0;
    public static final int REVPORT = 1;

  }

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
    public static final int kOPERATOR_CONTROLLER_PORT = 1;
    public static final int kAUTONOMOUS_CONSOLE_PORT = 2;
  }

  public static class ConsoleConstants {
    public static final int kPLACE_GAMEPIECE_SW = 1;
    public static final int kDRIVE_PATTERN_1_SW = 2;
  }
}
