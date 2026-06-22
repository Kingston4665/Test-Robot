// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EncoderIDs;
import frc.robot.Constants.EncoderOffsets;
import frc.robot.Constants.SparkMaxIDs;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.utils.AngleUtil;
import frc.robot.utils.InputUtil;
import frc.robot.utils.SwerveModule;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  // Sparkmaxes/motors for each swerve module
  SparkMax frontLeftDrive = new SparkMax(SparkMaxIDs.FRONT_LEFT_DRIVE, MotorType.kBrushless);
  SparkMax frontRightDrive = new SparkMax(SparkMaxIDs.FRONT_RIGHT_DRIVE, MotorType.kBrushless);
  SparkMax backLeftDrive = new SparkMax(SparkMaxIDs.BACK_LEFT_DRIVE, MotorType.kBrushless);
  SparkMax backRightDrive = new SparkMax(SparkMaxIDs.BACK_RIGHT_DRIVE, MotorType.kBrushless);

  SparkMax frontLeftTurn = new SparkMax(SparkMaxIDs.FRONT_LEFT_TURN, MotorType.kBrushless);
  SparkMax frontRightTurn = new SparkMax(SparkMaxIDs.FRONT_RIGHT_TURN, MotorType.kBrushless);
  SparkMax backLeftTurn = new SparkMax(SparkMaxIDs.BACK_LEFT_TURN, MotorType.kBrushless);
  SparkMax backRightTurn = new SparkMax(SparkMaxIDs.BACK_RIGHT_TURN, MotorType.kBrushless);

  // Gyroscope
  PigeonIMU gyro = new PigeonIMU(18);

  // Encoders
  AnalogEncoder frontLeftEncoder = new AnalogEncoder(EncoderIDs.FRONT_LEFT_ENCODER);
  AnalogEncoder frontRightEncoder = new AnalogEncoder(EncoderIDs.FRONT_RIGHT_ENCODER);
  AnalogEncoder backLeftEncoder = new AnalogEncoder(EncoderIDs.BACK_LEFT_ENCODER);
  AnalogEncoder backRightEncoder = new AnalogEncoder(EncoderIDs.BACK_RIGHT_ENCODER);

  AnalogInput frontLeftAnalogInput = new AnalogInput(EncoderIDs.FRONT_LEFT_ENCODER);
  AnalogInput frontRightAnalogInput = new AnalogInput(EncoderIDs.FRONT_RIGHT_ENCODER);
  AnalogInput backLeftAnalogInput = new AnalogInput(EncoderIDs.BACK_LEFT_ENCODER);
  AnalogInput backRightAnalogInput = new AnalogInput(EncoderIDs.BACK_RIGHT_ENCODER);

  RelativeEncoder frontLeftDriveEncoder = frontLeftDrive.getEncoder();
  RelativeEncoder frontRightDriveEncoder = frontRightDrive.getEncoder();
  RelativeEncoder backLeftDriveEncoder = backLeftDrive.getEncoder();
  RelativeEncoder backRightDriveEncoder = backRightDrive.getEncoder();

  // Swerve Modules
  SwerveModule frontLeft = new SwerveModule(frontLeftDrive, frontLeftTurn, frontLeftEncoder,
      frontLeftAnalogInput, frontLeftDriveEncoder, EncoderOffsets.FRONT_LEFT_ENCODER_OFFSET);
  SwerveModule frontRight = new SwerveModule(frontRightDrive, frontRightTurn, frontRightEncoder,
      frontRightAnalogInput, frontRightDriveEncoder, EncoderOffsets.FRONT_RIGHT_ENCODER_OFFSET);
  SwerveModule backLeft = new SwerveModule(backLeftDrive, backLeftTurn, backLeftEncoder,
      backLeftAnalogInput, backLeftDriveEncoder, EncoderOffsets.BACK_LEFT_ENCODER_OFFSET);
  SwerveModule backRight = new SwerveModule(backRightDrive, backRightTurn, backRightEncoder,
      backRightAnalogInput, backRightDriveEncoder, EncoderOffsets.BACK_RIGHT_ENCODER_OFFSET);

  // Kinematics & Odometry
  SwerveDriveKinematics kinematics;
  SwerveDriveOdometry odometry;

  /** Creates a new SwerveSubsytem. */
  public SwerveSubsystem() {

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW; // TODO LOWER THIS AT COMP, SLOWS COMPUTATION

    kinematics = new SwerveDriveKinematics(
      new Translation2d(0.368, 0.368),
      new Translation2d(0.368, -0.368),
      new Translation2d(-0.368, 0.368),
      new Translation2d(-0.368, -0.368)
		);

    gyro.setFusedHeading(0);

    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(Math.toRadians(gyro.getFusedHeading())),
        modulePositions());
  }

  public final SwerveModulePosition[] modulePositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };
  }

  public final SwerveModuleState[] moduleStates() {
    return new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()
    };
  }

	// Method to drive the robot, given x and y translation values, a turn value, and whether the driver wants field-relative control or turbo mode
  public void drive(double y, double x, double turn, boolean field, boolean turbo) {
    // Deadbands all of the values to prevent drift
    double dturn = InputUtil.deadband(turn);
    double speedMultiplier = SwerveDriveConstants.MAX_SPEED / (turbo ? 1 : 1.25); // Normal speed is 80%, turbo is 100%
    double forward = InputUtil.deadband(y) * speedMultiplier;
    double strafe = InputUtil.deadband(x) * speedMultiplier;
		
		// Makes the robot control field relative
		if (field == true) {
    double gyroRads = Math.toRadians(-gyro.getFusedHeading());
    double temp = forward * Math.cos(gyroRads) + strafe * Math.sin(gyroRads);
    strafe = -forward * Math.sin(gyroRads) + strafe * Math.cos(gyroRads);
    forward = temp;
		}

		// "dturn" is negative because the joystick positive direction and WPILib-positive rotation direction are opposite
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(forward, strafe, -dturn));
    setModuleStates(states);

		// Puts important information on the SmartDashboard for debugging and tuning purposes
    SmartDashboard.putNumber("Front Left Desired Angle", AngleUtil.circleMod(states[0].angle.getDegrees()));
    SmartDashboard.putNumber("Front Right Desired Angle", AngleUtil.circleMod(states[1].angle.getDegrees()));
    SmartDashboard.putNumber("Backleft Desired Angle", AngleUtil.circleMod(states[2].angle.getDegrees()));
    SmartDashboard.putNumber("Backright Desired Angle", AngleUtil.circleMod(states[3].angle.getDegrees()));

    SmartDashboard.putNumber("Front Left Power", states[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right Power", states[1].speedMetersPerSecond);
    SmartDashboard.putNumber("Back Left Power", states[2].speedMetersPerSecond);
    SmartDashboard.putNumber("Back Right Power", states[3].speedMetersPerSecond);

    SmartDashboard.putNumber("Front Left Angle", frontLeft.getAngle());
    SmartDashboard.putNumber("Front Right Angle", frontRight.getAngle());
    SmartDashboard.putNumber("Back Left Angle", backLeft.getAngle());
    SmartDashboard.putNumber("Back Right Angle", backRight.getAngle());

    SmartDashboard.putNumber("Front Left Speed", frontLeft.getSpeed());
    SmartDashboard.putNumber("Front Right Speed", frontRight.getSpeed());
    SmartDashboard.putNumber("Back Left Speed", backLeft.getSpeed());
    SmartDashboard.putNumber("Back Right Speed", backRight.getSpeed());

    SmartDashboard.putBoolean("Front Left Optimized?", frontLeft.isOptimized());
    SmartDashboard.putBoolean("Front Right Optimized?", frontRight.isOptimized());
    SmartDashboard.putBoolean("Back Left Optimized?", backLeft.isOptimized());
    SmartDashboard.putBoolean("Back Right Optimized?", backRight.isOptimized());

    SmartDashboard.putNumber("Pose X", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Pose Y", odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("Pose Degrees", getAngle());
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveDriveConstants.MAX_SPEED);

		// Optimizes the module states to prevent unnecessary rotation
    states[0] = optimize(states[0], Rotation2d.fromDegrees(frontLeft.getAngle()));
    states[1] = optimize(states[1], Rotation2d.fromDegrees(frontRight.getAngle()));
    states[2] = optimize(states[2], Rotation2d.fromDegrees(backLeft.getAngle()));
    states[3] = optimize(states[3], Rotation2d.fromDegrees(backRight.getAngle()));

		// Drives the modules by giving them the desired speed and angle
    frontLeft.drive(states[0].speedMetersPerSecond / SwerveDriveConstants.MAX_SPEED, states[0].angle.getDegrees());
    frontRight.drive(states[1].speedMetersPerSecond / SwerveDriveConstants.MAX_SPEED, states[1].angle.getDegrees());
    backLeft.drive(states[2].speedMetersPerSecond / SwerveDriveConstants.MAX_SPEED, states[2].angle.getDegrees());
    backRight.drive(states[3].speedMetersPerSecond / SwerveDriveConstants.MAX_SPEED, states[3].angle.getDegrees());
  }

  public void stopMotors() {
    frontLeft.stopMotors();
    frontRight.stopMotors();
    backLeft.stopMotors();
    backRight.stopMotors();
  }

  public void zeroWheels() {
    frontLeft.steerToAngle(0);
    frontRight.steerToAngle(0);
    backLeft.steerToAngle(0);
    backRight.steerToAngle(0);
  }

	public void antiPushWheels() {
    frontLeft.steerToAngle(45);
    frontRight.steerToAngle(315); // -45
    backLeft.steerToAngle(315); // -45
    backRight.steerToAngle(45);
  }

  public void resetEncoders() {
    frontLeftDriveEncoder.setPosition(0);
    frontRightDriveEncoder.setPosition(0);
    backLeftDriveEncoder.setPosition(0);
    backRightDriveEncoder.setPosition(0);
  }

  public void zeroGyro() {
    gyro.setFusedHeading(0);
  }

  public double getAngle() {
    return (gyro.getFusedHeading());
  }

  public SwerveDriveOdometry getOdometry() {
    return odometry;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public SwerveModule getFrontLeft() {
    return frontLeft;
  }

  public SwerveModule getFrontRight() {
    return frontRight;
  }

  public SwerveModule getBackLeft() {
    return backLeft;
  }

  public SwerveModule getBackRight() {
    return backRight;
  }

  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(
        -desiredState.speedMetersPerSecond,
        desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    } 
    else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }

	// Adds 180 degrees to the gyroscope
  public void allianceRelativeGyroscopeControl() {
    gyro.setFusedHeading(getAngle() + 180);
  }

	// Returns the pose of the of the robot
	public Pose2d getPose() {
  	return odometry.getPoseMeters();
	}

	// Resets the odometry to a given pose
	public void resetPose(Pose2d pose) {
  	odometry.resetPosition(
      Rotation2d.fromDegrees(getAngle()),
      modulePositions(),
      pose
  	);
	}

	public void driveRobotRelative(ChassisSpeeds speeds) {
    setModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(moduleStates());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

		// Updates the odometry
    odometry.update(Rotation2d.fromDegrees(getAngle()), modulePositions());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
