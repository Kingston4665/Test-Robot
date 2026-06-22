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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.SparkMaxIDs;

public class Roller extends SubsystemBase {

  // These are the class members
  SparkMax rollerMax = new SparkMax(SparkMaxIDs.ROLLERS, MotorType.kBrushless);

  /** Creates a new Roller. */
  public Roller() {
    SparkMaxConfig RollerConfig = new SparkMaxConfig();
    RollerConfig.smartCurrentLimit(40);
    rollerMax.configure(RollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
 
  // This method runs the roller
  public void rollerOn() {
    rollerMax.set(RollerConstants.ROLLER_SPEED);
  }

  // This command runs the roller
  public Command rollerOnCommand() {
    return run(() -> {
      rollerMax.set(RollerConstants.ROLLER_SPEED);
    });
  }

  // This method runs the roller in reverse
  public void rollerReverse() {
    rollerMax.set(-RollerConstants.ROLLER_SPEED);
  }

  // This command runs the roller in reverse
  public Command rollerReverseCommand() {
    return run(() -> {
      rollerMax.set(-RollerConstants.ROLLER_SPEED);
    });
  }

  // This command runs the rollers forwards then backwards repeatedly to keep fuel moving
  public Command jiggleRollerCommand() {
    return new SequentialCommandGroup(
        runOnce(() -> rollerMax.set(-RollerConstants.ROLLER_GIGGLE_SPEED)),
        new WaitCommand(0.5),

        runOnce(() -> rollerMax.set(RollerConstants.ROLLER_GIGGLE_SPEED)),
        new WaitCommand(0.5)
    ).repeatedly();
  }

  // This method stops the roller
  public void rollerOff() {
    rollerMax.set(0);
  }

  // This command stops the roller
  public Command rollerOffCommand() {
    return run(() -> {
      rollerMax.set(0);
    });
  }

  public Command rollerCommand(CommandXboxController driverController, CommandXboxController copilotController) {
    return run(() -> {

      if (driverController.leftBumper().getAsBoolean() || copilotController.leftBumper().getAsBoolean()) {
        rollerOff();
      }

      if (driverController.rightBumper().getAsBoolean() || copilotController.rightBumper().getAsBoolean()) {
        rollerOn();
      }

    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
