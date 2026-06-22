// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.SparkMaxIDs;

public class Kicker extends SubsystemBase {

  // These are the class members
  SparkMax kickerMax = new SparkMax(SparkMaxIDs.KICKER, MotorType.kBrushless);

  /** Creates a new Kicker. */
  public Kicker() {

    // Kicker
    SparkMaxConfig KickerConfig = new SparkMaxConfig();
    KickerConfig.smartCurrentLimit(40);
    kickerMax.configure(KickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // This method runs the kicker
  public void kickerOn() {
    kickerMax.set(KickerConstants.KICKER_SPEED);
  }

  // This command runs the kicker
  public Command kickerOnCommand() {
    return run(() -> {
      kickerMax.set(KickerConstants.KICKER_SPEED);
    });
  }

  // This method runs the kicker in reverse
  public void kickerReverse() {
    kickerMax.set(-KickerConstants.KICKER_SPEED);
  }

  // This command runs the kicker in reverse
  public Command kickerReverseCommand() {
    return run(() -> {
      kickerMax.set(-KickerConstants.KICKER_SPEED);
    });
  }

  // This method turns the kicker off
  public void kickerOff() {
    kickerMax.set(0);
  }

  // This command turns the kicker off
  public Command kickerOffCommand() {
    return run(() -> {
      kickerMax.set(0);
    });
  }

  public Command kickerCommand(CommandXboxController driverController, CommandXboxController copilotController) {
    return run(() -> {

      if (driverController.leftBumper().getAsBoolean() || copilotController.leftBumper().getAsBoolean()) {
        kickerOff();
      }
      
      if (driverController.rightBumper().getAsBoolean() || copilotController.rightBumper().getAsBoolean()) {
        kickerOn();
      }

    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}