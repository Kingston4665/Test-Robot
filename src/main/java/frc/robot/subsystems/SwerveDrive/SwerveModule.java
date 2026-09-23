package frc.robot.subsystems.SwerveDrive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.math.MathUtil.clamp;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.AngleUtil;

public class SwerveModule {

  private final SparkMax drive;
  private final SparkMax turn;

  private final AnalogEncoder encoder;
  private final RelativeEncoder driveEncoder;

  private double encoderOffset;

  /**
   * Constructs a new swerve module
   * 
   * @param drive        motor controller for drive motor
   * @param turn         motor controller for steer motor
   * @param steerEncoder steer encoder - not from spark max
   * @param angleOffset  module angle offset
   */
  public SwerveModule(SparkMax drive, SparkMax turn, AnalogEncoder encoder,
      RelativeEncoder driveEncoder, double encoderOffset) {
    this.drive = drive;
    this.turn = turn;
    this.encoder = encoder;
    this.encoderOffset = encoderOffset;
    this.driveEncoder = driveEncoder;
  }

  /**
   * Drive the swerve module with power and angle
   * 
   * @param power Motor power of the swerve module
   * @param angle Desired angle
   */
  public void drive(double power, double angle) {
    drive.set(power);
    steerToAngle(angle);
  }

  public void stopMotors() {
    drive.stopMotor();
    turn.stopMotor();
  }

  /**
   * Steer swerve module to angle between [0,360)
   * 
   * @param angle the desired angle of the swerve module
   */
  public void steerToAngle(double angle) {
    steerToAngle(angle, 0.008);
  }

  /**
   * Steer swerve module to angle between [0,360)
   * 
   * @param angle the desired angle of the swerve module
   * @param p     the p value to use
   */
  public void steerToAngle(double angle, double p) {
    double error = angle - getAngle();
    if (error < -180) error += 360;
    if (error > 180) error -= 360;

    // Stop small corrections so the wheel does not keep twitching.
    if (Math.abs(error) <= 1.25) {
      turn.stopMotor();
      return;
    }
    double output = clamp(error * p / 3.0, -1.0 / 3.0, 1.0 / 3.0);
    if (Math.abs(output) < 0.015) output = Math.copySign(0.015, output);
    turn.set(output);
  }

  // thanks 2910, very cool

  /**
   * Gets swerve module angle
   * 
   * @return the swerve module angle
   */
  public double getAngle() {
		// The working 2026 configuration does not invert the absolute encoder.
    double angle = encoder.get() * 360;
		
    angle = AngleUtil.circleMod(angle - encoderOffset);

    return angle;
  }

  /**
   * Gets the state for odometry
   * 
   * @return the swerve module state
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      (driveEncoder.getVelocity() / SwerveConstants.GEAR_RATIO) * SwerveConstants.WHEEL_DIAMETER * Math.PI / 60,
      Rotation2d.fromDegrees(getAngle())
		);
  }

  public SparkMax getDrive() {
    return drive;
  }

  public SparkMax getTurn() {
    return turn;
  }

  public AnalogEncoder getEncoder() {
    return encoder;
  }

  public double getEncoderOffset() {
    return encoderOffset;
  }

  public void setEncoderOffset(double encoderOffset) {
    this.encoderOffset = encoderOffset;
  }

  public double getSpeed() {
    return drive.get();
  }

  public SwerveModulePosition getPosition() {
    double meters = (driveEncoder.getPosition() / SwerveConstants.GEAR_RATIO)
        * SwerveConstants.WHEEL_DIAMETER * Math.PI;

    return new SwerveModulePosition(
      meters,
      Rotation2d.fromDegrees(getAngle())
		);
  }
}
