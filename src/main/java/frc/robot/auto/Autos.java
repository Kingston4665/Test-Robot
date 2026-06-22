// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AngleToHub;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LimelightSubsytem;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;
import frc.robot.utils.PathPlannerUtils;

/** Add your docs here. */
public class Autos {

	private final SwerveSubsystem swerveSubsystem;
	private final Intake intake;
	private final Kicker kicker;
	private final LimelightSubsytem limelightSubsytem;
	private final Roller roller;
	private final Shooter shooter;

	public Autos(SwerveSubsystem swerveSubsystem, Intake intake, Kicker kicker, LimelightSubsytem limelightSubsystem, Roller roller, Shooter shooter) {
		this.swerveSubsystem = swerveSubsystem;
		this.intake = intake;
		this.kicker = kicker;
		this.limelightSubsytem = limelightSubsystem;
		this.roller = roller;
		this.shooter = shooter;
	}

	// Auto Commands
	public Command followPath(Optional<PathPlannerPath> path) {
    return AutoBuilder.followPath(path.get());
  }

	public Command shoot() {
		return Commands.parallel(
			shooter.shooterOnCommand(),
			kicker.kickerOnCommand(),
			roller.rollerReverseCommand().withTimeout(0.5)
				.andThen(roller.rollerOnCommand())
		);
	}

	public Command intake() {
		return Commands.sequence(
			intake.intakeLiftDownCommand().withTimeout(1),
			intake.intakeSpinnyCommand()
		);
	}

	// Autos
	public Command basicAuto() {
		Optional<PathPlannerPath> backUp = PathPlannerUtils.loadPathByName("First 8 Path");

		PathPlannerAuto auto;

		var cmd = backUp.isEmpty() ? Commands.none() : Commands.sequence(
			followPath(backUp),
			shoot().withTimeout(4)
		);

		auto = new PathPlannerAuto(cmd);
		return auto;
	}

	public Command depotAuto() {
		Optional<PathPlannerPath> backUp = PathPlannerUtils.loadPathByName("First 8 Path");
		Optional<PathPlannerPath> toDepot = PathPlannerUtils.loadPathByName("Center To Depot");
		Optional<PathPlannerPath> backToHub = PathPlannerUtils.loadPathByName("Depot Back To Center");


		PathPlannerAuto auto;

		var cmd = backUp.isEmpty() || toDepot.isEmpty() || backToHub.isEmpty()
			? Commands.none() : Commands.sequence(
					followPath(backUp),
					shoot().withTimeout(4),
					followPath(toDepot),
					intake().withTimeout(4),
					followPath(backToHub),
					new AngleToHub(swerveSubsystem, limelightSubsytem).withTimeout(1),
					shoot().withTimeout(6)
		);

		auto = new PathPlannerAuto(cmd);
		return auto;
	}
}
