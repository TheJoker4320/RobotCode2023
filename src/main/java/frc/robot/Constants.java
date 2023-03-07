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
  public static class ChassisConstants {
    public static final int FRONT_RIGHT_MOTOR_PORT = 7;
    public static final int BACK_RIGHT_MOTOR_PORT = 8;
    public static final int FRONT_LEFT_MOTOR_PORT = 9;
    public static final int BACK_LEFT_MOTOR_PORT = 10;

    public static final boolean SET_INVERTED = true;
    public static final int[] LEFT_ENCODER_SCORE = { 6, 7 };
    public static final int[] RIGHT_ENCODER_SCORE = { 9, 8 };
  }

  public static class OperatorConstants {
    public static final int kBUTTONS_PORT = 0;
    public static final int kDRIVING_PORT = 1;

    public static final int SHIFTER_BUTTON = 11;
    public static final int COLLECT_FACING_BUTTON = 7;
    public static final int COLLECT_AGAINST_BUTTON = 8;
    public static final int RAMP_BUTTON = 9;
  }

  public static class PIDConstants {
    public static final String kOPTION_STABALIZE_RAMP = "Ramp Stabalizing";
    public static final String kOPTION_DRIVE_TO_PIECES = "Drive to pieces";
    public static final String kOPTION_TEST = "Autonomous testing";

    public static final double kP_DRIVING = 1.15;
    public static final double kI_DRIVING = 0;
    public static final double kD_DRIVING = 0.1;

    public static final double SET_POINT = 2; // Its in meters
    public static final double TOLERANCE_DRIVING = 0.03;

    /*
     * Ramp constants - PID whise
     */

    public static final double kP_RAMP = 0.03;
    public static final double kI_RAMP = 0;
    public static final double kD_RAMP = 0.007;

    public static final double RAMP_MAX_SPEED = 0.6;

    public static final double TOLERANCE_RAMP = 1.5;

    public static final double GEAR_RATIO = 27.0 / (double) 5.0;

    public static final int ENCODER_TICKS = 256;
    public static final double CIRCUMFERENCE = Math.PI * Units.inchesToMeters(6);
    public static final double GEAR_RATIO_TO_TICKS = GEAR_RATIO * ENCODER_TICKS;

    public static final double TICKS_TO_METERS = CIRCUMFERENCE / GEAR_RATIO_TO_TICKS; // Used to convert a tick to the
                                                                                      // equivelent in meters

    public static final double kDISTANCE_TO_RAMP = 2; //2.457m
    public static final double kDISTANCE_TO_GAMEPIECES = 5.65 - 0.4; //5.4356m

    public static final double kROBOT_LENGTH = 0.93;
  }

  public static class LimeLightConstants {
    public static final double kP = 0.06;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kTOLERANCE = 1;
  }

  public static class RamseteCommandConstatns {
    public static final double kS = 0.53418;
    public static final double kV = 2.916;
    public static final double kA = 4.3361;

    public static final double kTrackwidthMeters = 0.7; // 0.79
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static class PneumaticsConstants {
    public static final int FIRST_CLAW_SOLENOID_PORT = 5;
    public static final int SECOND_CLAW_SOLENOID_PORT = 4;

    public static final int FIRST_SHIFTER_SOLENOID_PORT = 6;
    public static final int SECOND_SHIFTER_SOLENOID_PORT = 7;
  }

  public final class ArmConstants {
    public static final int MASTER_PORT = 11;
    public static final int SLAVE_PORT = 12;

    public static final int MICRO_SWITCH_PORT = 1;
    public static final int ENCODER_CHANNEL = 0;

    public static final double kP_ARM = 0.03;
    public static final double kI_ARM = 0;
    public static final double kD_ARM = 0;

    public static final double kARM_SPEED = 0.5;
    public static final double kMAX_ARM_AUTONOMOUS_SPEED = 0.6;

    //Angle related constants

    public static final double kENCODER_ERROR = 297 - 32;

    public static final double kMAX_ARM_ANGLE = 120;
    public static final double kANGLE_SETPOINT = 109;
    public static final double kANGLE_TOLERANCE = 3;
  }

  public final class CollectorConstants {
    public static final int FRONT_MOTOR_PORT = 6;
    public static final double COLLECTING_SPEED = 0.9;
  }
}