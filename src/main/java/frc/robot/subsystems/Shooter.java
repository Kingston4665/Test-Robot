// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SparkMaxIDs;

public class Shooter extends SubsystemBase {

  // These are the class members
  SparkMax shooterMax = new SparkMax(SparkMaxIDs.SHOOTER, MotorType.kBrushless);

  RelativeEncoder shooterEncoder = shooterMax.getEncoder();

	double shooterRPM;
  double shooterPosition;

  /** Creates a new Shooter. */
  public Shooter() {
    SparkMaxConfig ShooterConfig = new SparkMaxConfig();
    ShooterConfig.smartCurrentLimit(40);
    shooterMax.configure(ShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // This command makes the shooter go weeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee ;)
  public void shooterOn() {
    shooterMax.set(ShooterConstants.SHOOTER_SPEED);
  }

  // This command makes the shooter go weeee
  public Command shooterOnCommand() {
    return run(() -> {
      shooterMax.set(ShooterConstants.SHOOTER_SPEED);
    });
  }


  // This method turns the the shooter off
  public void shooterOff() {
    shooterMax.set(0);
  }

  // This command turns the the shooter off
  public Command shooterOffCommand() {
    return run(() -> {
      shooterMax.set(0);
    });
  }

// 
  public Command shooterCommand(CommandXboxController driverController, CommandXboxController copilotController) {
    return run(() -> {

      if (driverController.leftBumper().getAsBoolean() || copilotController.leftBumper().getAsBoolean()) {
        shooterOff();
      }

      if (driverController.rightBumper().getAsBoolean() || copilotController.rightBumper().getAsBoolean()) {
        shooterOn();
      }

    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterRPM = shooterEncoder.getVelocity();
    shooterPosition = shooterEncoder.getPosition();
    SmartDashboard.putNumber("Shooter RPM", shooterRPM);
    SmartDashboard.putNumber("Shooter Position", shooterPosition);
  }
}
