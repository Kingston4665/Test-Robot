// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsytem;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AngleToHub extends Command {

	private final SwerveSubsystem swerveSubsystem;
	private final LimelightSubsytem limelightSubsystem;

  /** Creates a new AimAtHub. */
  public AngleToHub(SwerveSubsystem swerveSubsystem, LimelightSubsytem limelightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
		this.swerveSubsystem = swerveSubsystem;
		this.limelightSubsystem = limelightSubsystem;
    addRequirements(swerveSubsystem, limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
		swerveSubsystem.drive(0, 0, limelightSubsystem.getRotationRate(), false, false);
	}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
		swerveSubsystem.drive(0, 0, 0, false, false);
	}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
