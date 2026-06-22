// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class OperatorConstants {
    public static final int DRIVER = 0; // KEEP ZEROOOOOOOOO!!!!!!!!
    public static final int COPILOT = 1; // Keep One
    public static final double DEADBAND = 0.1;
    public static final DriverStation.Alliance RED_ALLIANCE = DriverStation.Alliance.Red;
    public static final DriverStation.Alliance BLUE_ALLIANCE = DriverStation.Alliance.Blue;
  }

  public static final class SwerveDriveConstants {
    public static final double MAX_SPEED = 5.0; // Meters per second
  }
  
  public static final class SparkMaxIDs {
		public static final int FRONT_LEFT_DRIVE = 1;
    public static final int FRONT_RIGHT_DRIVE = 2;
    public static final int BACK_LEFT_DRIVE = 3;
    public static final int BACK_RIGHT_DRIVE = 4;
    public static final int FRONT_LEFT_TURN = 5;
    public static final int FRONT_RIGHT_TURN = 6;
    public static final int BACK_LEFT_TURN = 7;
    public static final int BACK_RIGHT_TURN = 8;
    public static final int SHOOTER = 9;
    public static final int KICKER = 10;
    public static final int ROLLERS = 11;
    public static final int INTAKE_LIFT = 12;
    public static final int INTAKE_SPINNY = 13;
    // public static final int LEFT_CLIMBER = 16; // Currently not on the robot
    // public static final int RIGHT_CLIMBER = 17; // Currently not on the robot
  }

	public static final class EncoderIDs {
    public static final int FRONT_LEFT_ENCODER = 0;
    public static final int FRONT_RIGHT_ENCODER = 1;
    public static final int BACK_LEFT_ENCODER = 2;
    public static final int BACK_RIGHT_ENCODER = 3;
  }

  public static final class EncoderOffsets {
    public static final double FRONT_LEFT_ENCODER_OFFSET = 0;
    public static final double FRONT_RIGHT_ENCODER_OFFSET = 0;
    public static final double BACK_LEFT_ENCODER_OFFSET = 0;
    public static final double BACK_RIGHT_ENCODER_OFFSET = 0;
  }

  public static final class SwerveConstants {
    public static final double GEAR_RATIO = 0;
    public static final double WHEEL_DIAMETER = 0; // In meters
  }

  public static final class ShooterConstants {
    public static final double SHOOTER_SPEED = -1; // Negative to make the shooter go in the correct direction
  }

  public static final class IntakeConstants {
    public static final double LIFT_SPEED = 0.4;
    public static final double SPIN_SPEED = 1;
  }

  public static final class ClimberConstants {
    public static final double CLIMBER_SPEED = -0.5; // Negative to make the climber go in the correct direction
  }

  public static final class KickerConstants {
    public static final double KICKER_SPEED = -1; // Negative to make the kicker go in the correct direction
  }

  public static final class RollerConstants {
    public static final double ROLLER_SPEED = 1;
    public static final double ROLLER_GIGGLE_SPEED = 0.5;
  }

  public static final class LimelightConstants {
    public static final double AIM_KP = 0.04; // Aiming sensitivity: 0.04 is smooth and prevents battery brownouts
    public static final String LIME_LIGHT_NAME = "limelight-oasis"; // This MUST match the name in your Limelight Web Dashboard exactly
  }
}
