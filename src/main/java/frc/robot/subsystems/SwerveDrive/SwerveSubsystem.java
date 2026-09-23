// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EncoderIDs;
import frc.robot.Constants.EncoderOffsets;
import frc.robot.Constants.SparkMaxIDs;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.utils.AngleUtil;
import frc.robot.utils.InputUtil;

public class SwerveSubsystem extends SubsystemBase {

  // Sparkmaxes/motors for each swerve module
  private final SparkMax frontLeftDrive = new SparkMax(SparkMaxIDs.FRONT_LEFT_DRIVE, MotorType.kBrushless);
  private final SparkMax frontRightDrive = new SparkMax(SparkMaxIDs.FRONT_RIGHT_DRIVE, MotorType.kBrushless);
  private final SparkMax backLeftDrive = new SparkMax(SparkMaxIDs.BACK_LEFT_DRIVE, MotorType.kBrushless);
  private final SparkMax backRightDrive = new SparkMax(SparkMaxIDs.BACK_RIGHT_DRIVE, MotorType.kBrushless);

  private final SparkMax frontLeftTurn = new SparkMax(SparkMaxIDs.FRONT_LEFT_TURN, MotorType.kBrushless);
  private final SparkMax frontRightTurn = new SparkMax(SparkMaxIDs.FRONT_RIGHT_TURN, MotorType.kBrushless);
  private final SparkMax backLeftTurn = new SparkMax(SparkMaxIDs.BACK_LEFT_TURN, MotorType.kBrushless);
  private final SparkMax backRightTurn = new SparkMax(SparkMaxIDs.BACK_RIGHT_TURN, MotorType.kBrushless);

  // Gyroscope
  private final Pigeon2 gyro = new Pigeon2(18);

  // Encoders
  private final AnalogEncoder frontLeftEncoder = new AnalogEncoder(EncoderIDs.FRONT_LEFT_ENCODER);
  private final AnalogEncoder frontRightEncoder = new AnalogEncoder(EncoderIDs.FRONT_RIGHT_ENCODER);
  private final AnalogEncoder backLeftEncoder = new AnalogEncoder(EncoderIDs.BACK_LEFT_ENCODER);
  private final AnalogEncoder backRightEncoder = new AnalogEncoder(EncoderIDs.BACK_RIGHT_ENCODER);

  private final RelativeEncoder frontLeftDriveEncoder = frontLeftDrive.getEncoder();
  private final RelativeEncoder frontRightDriveEncoder = frontRightDrive.getEncoder();
  private final RelativeEncoder backLeftDriveEncoder = backLeftDrive.getEncoder();
  private final RelativeEncoder backRightDriveEncoder = backRightDrive.getEncoder();

  // Swerve Modules
  private final SwerveModule frontLeft = new SwerveModule(frontLeftDrive, frontLeftTurn, frontLeftEncoder,
      frontLeftDriveEncoder, EncoderOffsets.FRONT_LEFT_ENCODER_OFFSET);
  private final SwerveModule frontRight = new SwerveModule(frontRightDrive, frontRightTurn, frontRightEncoder,
      frontRightDriveEncoder, EncoderOffsets.FRONT_RIGHT_ENCODER_OFFSET);
  private final SwerveModule backLeft = new SwerveModule(backLeftDrive, backLeftTurn, backLeftEncoder,
      backLeftDriveEncoder, EncoderOffsets.BACK_LEFT_ENCODER_OFFSET);
  private final SwerveModule backRight = new SwerveModule(backRightDrive, backRightTurn, backRightEncoder,
      backRightDriveEncoder, EncoderOffsets.BACK_RIGHT_ENCODER_OFFSET);

  // Kinematics & Odometry
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;
  private boolean motorConfigurationValid = true;

  /** Creates a new SwerveSubsytem. */
  public SwerveSubsystem() {

    // SwerveModule does the wheel conversion itself, so use raw motor rotations and RPM.
    SparkMaxConfig driveEncoderConfig = new SparkMaxConfig();
    driveEncoderConfig.encoder.positionConversionFactor(1.0).velocityConversionFactor(1.0);
    checkConfiguration("FL drive", frontLeftDrive.configure(driveEncoderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    checkConfiguration("FR drive", frontRightDrive.configure(driveEncoderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    checkConfiguration("BL drive", backLeftDrive.configure(driveEncoderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    checkConfiguration("BR drive", backRightDrive.configure(driveEncoderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));

    // The working 2026 module configuration inverts every angle motor.
    SparkMaxConfig turnMotorConfig = new SparkMaxConfig();
    turnMotorConfig.inverted(true);
    checkConfiguration("FL turn", frontLeftTurn.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    checkConfiguration("FR turn", frontRightTurn.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    checkConfiguration("BL turn", backLeftTurn.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    checkConfiguration("BR turn", backRightTurn.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    kinematics = new SwerveDriveKinematics(
			new Translation2d(SwerveConstants.MODULE_OFFSET_FROM_CENTER, SwerveConstants.MODULE_OFFSET_FROM_CENTER),
			new Translation2d(SwerveConstants.MODULE_OFFSET_FROM_CENTER, -SwerveConstants.MODULE_OFFSET_FROM_CENTER),
			new Translation2d(-SwerveConstants.MODULE_OFFSET_FROM_CENTER, SwerveConstants.MODULE_OFFSET_FROM_CENTER),
			new Translation2d(-SwerveConstants.MODULE_OFFSET_FROM_CENTER, -SwerveConstants.MODULE_OFFSET_FROM_CENTER)
		);

    gyro.setYaw(0);

    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(Math.toRadians(gyro.getYaw().getValueAsDouble())),
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
    double speedMultiplier = SwerveDriveConstants.MAX_SPEED / (turbo ? 1 : 1.25); // Normal speed is 80%, turbo is 100%
    double angularSpeedMultiplier = SwerveDriveConstants.MAX_ANGULAR_SPEED / (turbo ? 1 : 1.25);
    double dturn = InputUtil.deadband(turn) * angularSpeedMultiplier;
    double forward = InputUtil.deadband(y) * speedMultiplier;
    double strafe = InputUtil.deadband(x) * speedMultiplier;

    // Stop the drive and steering motors while all controls are centered. Without
    // this guard, zero-speed kinematics commands every module to turn to 0 degrees.
    if (forward == 0.0 && strafe == 0.0 && dturn == 0.0) {
      stopMotors();
      return;
    }
		
		// The joystick turn direction is opposite WPILib's positive rotation.
		ChassisSpeeds speeds = new ChassisSpeeds(forward, strafe, -dturn);
		if (field) {
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromDegrees(getAngle()));
		}
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    setModuleStates(states);

  }

  private void checkConfiguration(String motorName, REVLibError result) {
    if (result != REVLibError.kOk) {
      motorConfigurationValid = false;
      DriverStation.reportError("Swerve " + motorName + " configuration failed: " + result, false);
    }
  }

  public void setModuleStates(SwerveModuleState[] states) {
    if (!motorConfigurationValid) {
      stopMotors();
      return;
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveDriveConstants.MAX_SPEED);

		// Optimizes the module states to prevent unnecessary rotation
    states[0] = optimizeAndScale(states[0], Rotation2d.fromDegrees(frontLeft.getAngle()));
    states[1] = optimizeAndScale(states[1], Rotation2d.fromDegrees(frontRight.getAngle()));
    states[2] = optimizeAndScale(states[2], Rotation2d.fromDegrees(backLeft.getAngle()));
    states[3] = optimizeAndScale(states[3], Rotation2d.fromDegrees(backRight.getAngle()));

		// Drives the modules by giving them the desired speed and angle
    frontLeft.drive(states[0].speedMetersPerSecond / SwerveDriveConstants.MAX_SPEED, states[0].angle.getDegrees());
    frontRight.drive(states[1].speedMetersPerSecond / SwerveDriveConstants.MAX_SPEED, states[1].angle.getDegrees());
    backLeft.drive(states[2].speedMetersPerSecond / SwerveDriveConstants.MAX_SPEED, states[2].angle.getDegrees());
    backRight.drive(states[3].speedMetersPerSecond / SwerveDriveConstants.MAX_SPEED, states[3].angle.getDegrees());

    // The same command telemetry is available in teleop and autonomous.
    SmartDashboard.putNumber("FL Commanded mps", states[0].speedMetersPerSecond);
    SmartDashboard.putNumber("FR Commanded mps", states[1].speedMetersPerSecond);
    SmartDashboard.putNumber("BL Commanded mps", states[2].speedMetersPerSecond);
    SmartDashboard.putNumber("BR Commanded mps", states[3].speedMetersPerSecond);
    SmartDashboard.putNumber("FL Target Angle", AngleUtil.circleMod(states[0].angle.getDegrees()));
    SmartDashboard.putNumber("FR Target Angle", AngleUtil.circleMod(states[1].angle.getDegrees()));
    SmartDashboard.putNumber("BL Target Angle", AngleUtil.circleMod(states[2].angle.getDegrees()));
    SmartDashboard.putNumber("BR Target Angle", AngleUtil.circleMod(states[3].angle.getDegrees()));
  }

  public void stopMotors() {
    frontLeft.stopMotors();
    frontRight.stopMotors();
    backLeft.stopMotors();
    backRight.stopMotors();
    SmartDashboard.putNumber("FL Commanded mps", 0.0);
    SmartDashboard.putNumber("FR Commanded mps", 0.0);
    SmartDashboard.putNumber("BL Commanded mps", 0.0);
    SmartDashboard.putNumber("BR Commanded mps", 0.0);
  }

  public void zeroWheels() {
    frontLeft.steerToAngle(0);
    frontRight.steerToAngle(0);
    backLeft.steerToAngle(0);
    backRight.steerToAngle(0);
  }

  public boolean wheelsAreZero() {
    return AngleUtil.distance(frontLeft.getAngle(), 0) <= SwerveConstants.WHEEL_ZERO_TOLERANCE_DEGREES
        && AngleUtil.distance(frontRight.getAngle(), 0) <= SwerveConstants.WHEEL_ZERO_TOLERANCE_DEGREES
        && AngleUtil.distance(backLeft.getAngle(), 0) <= SwerveConstants.WHEEL_ZERO_TOLERANCE_DEGREES
        && AngleUtil.distance(backRight.getAngle(), 0) <= SwerveConstants.WHEEL_ZERO_TOLERANCE_DEGREES;
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
    gyro.setYaw(0);
  }

  public double getAngle() {
    return (gyro.getYaw().getValueAsDouble());
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

  private static SwerveModuleState optimizeAndScale(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    // A zero-speed command has no meaningful angle; keep the wheels where they
    // are instead of turning all four to zero at the end of every path.
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
      return new SwerveModuleState(0.0, currentAngle);
    }

    SwerveModuleState optimized = optimize(desiredState, currentAngle);
    // Do not drive sideways while a module is still steering toward its target.
    double angleError = optimized.angle.minus(currentAngle).getRadians();
    return new SwerveModuleState(
        optimized.speedMetersPerSecond * Math.max(0.0, Math.cos(angleError)),
        optimized.angle);
  }

	// Adds 180 degrees to the gyroscope
  public void allianceRelativeGyroscopeControl() {
    gyro.setYaw(getAngle() + 180);
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

    // Publish the measured values in auto too. Compare each measured speed and
    // angle with its command; optimization can intentionally reverse a wheel.
    // The robot-relative +X check should still increase pose X at zero heading.
    SmartDashboard.putNumber("Pose X", getPose().getX());
    SmartDashboard.putNumber("Pose Y", getPose().getY());
    SmartDashboard.putNumber("Pose Degrees", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Gyro Yaw", getAngle());
    SmartDashboard.putNumber("FL Measured mps", frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("FR Measured mps", frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("BL Measured mps", backLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("BR Measured mps", backRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("FL Measured Angle", frontLeft.getAngle());
    SmartDashboard.putNumber("FR Measured Angle", frontRight.getAngle());
    SmartDashboard.putNumber("BL Measured Angle", backLeft.getAngle());
    SmartDashboard.putNumber("BR Measured Angle", backRight.getAngle());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
