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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SparkMaxIDs;

public class Intake extends SubsystemBase {

  // These are the class members
  SparkMax intakeLiftMax = new SparkMax(SparkMaxIDs.INTAKE_LIFT, MotorType.kBrushless);
  SparkMax intakeSpinnyMax = new SparkMax(SparkMaxIDs.INTAKE_SPINNY, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {
    SparkMaxConfig IntakeLiftConfig = new SparkMaxConfig();
    IntakeLiftConfig.smartCurrentLimit(40);
    intakeLiftMax.configure(IntakeLiftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig IntakeSpinnyConfig = new SparkMaxConfig();
    IntakeSpinnyConfig.smartCurrentLimit(40);
    intakeSpinnyMax.configure(IntakeSpinnyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // This method lowers the intake lift
  public void intakeLiftDown() {
    intakeLiftMax.set(IntakeConstants.LIFT_SPEED);
  }

  // This command lowers the intake lift
  public Command intakeLiftDownCommand() {
    return run(() -> {
      intakeLiftMax.set(IntakeConstants.LIFT_SPEED);
    });
  }

  // This method raises the intake lift
  public void intakeLiftUp() {
    intakeLiftMax.set(-IntakeConstants.LIFT_SPEED);
  }

  // This command raises the intake lift
  public Command intakeLiftUpCommand() {
    return run(() -> {
      intakeLiftMax.set(-IntakeConstants.LIFT_SPEED);
    });
  }

  // This method makes the intake go spinny
  public void intakeSpinny() {
    intakeSpinnyMax.set(IntakeConstants.SPIN_SPEED);
  }

  // This command makes the intake go spinnyyyyyyyyyyyyyyyyyyyyyyyyyyyyy :)
  public Command intakeSpinnyCommand() {
    return run(() -> {
      intakeSpinnyMax.set(IntakeConstants.SPIN_SPEED);
    });
  }

  // This method makes the intake go reverse spinny
  public void intakeReverseSpinny() {
    intakeSpinnyMax.set(-IntakeConstants.SPIN_SPEED);
  }

  // This command makes the intake go reverse spinny
  public Command intakeReverseCommand() {
    return run(() -> {
      intakeSpinnyMax.set(-IntakeConstants.SPIN_SPEED);
    });
  }

  // This method makes the intake stop going spinny
  public void intakeStopSpinny() {
    intakeSpinnyMax.set(0);
  }

  // This command makes the intake stop going spinny
  public Command intakeStopSpinnyCommand() {
    return run(() -> {
      intakeSpinnyMax.set(0);
    });
  }

  // Command that allows joystick control of the intake lift and runs intake in reverse when the intake moves upward
  public Command intakeCommand(CommandXboxController copilotController) {
    return run(() -> {

      if (Math.abs(copilotController.getLeftY()) > OperatorConstants.DEADBAND) {
        intakeLiftMax.set(-copilotController.getLeftY() * IntakeConstants.LIFT_SPEED);
      } 
      else if (copilotController.b().getAsBoolean()) {
       intakeStopSpinny();
      }
      else if (copilotController.x().getAsBoolean()) {
        intakeSpinny();
      }
      else if (copilotController.y().getAsBoolean()) {
        intakeReverseSpinny();
      }
      else {
        intakeLiftMax.set(0);
      }
     
    });
  }


  // Command that allows joystick control of the intake lift and runs intake in reverse when the intake moves upward
  public Command oneControllerIntakeCommand(CommandXboxController driverController) {
    return run(() -> {

      if (driverController.leftTrigger().getAsBoolean()) {
        intakeLiftUp();
        // intakeReverseSpinny(); // May cause problems, comment out if needed
      } 
      else if (driverController.rightTrigger().getAsBoolean()) {
        intakeLiftDown();
      }
      else if (driverController.b().getAsBoolean()) {
        intakeStopSpinny();
      }
      else if (driverController.x().getAsBoolean()) {
        intakeSpinny();
      }
      else if (driverController.y().getAsBoolean()) {
        intakeReverseSpinny();
      }
      else {
        intakeLiftMax.set(0);
      }
    
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
