// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.LockOnToHub;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LimelightSubsytem;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. 
 * Since Command-based is a "declarative" paradigm, very little 
 * robot logic should actually be handled in the {@link Robot} 
 * periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, 
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final SwerveSubsystem driveBase = new SwerveSubsystem();
	private final Shooter shooter = new Shooter();
	private final Intake intake = new Intake();
	private final Kicker kicker = new Kicker();
	private final Roller roller = new Roller();
	private final LimelightSubsytem limelightSubsytem = new LimelightSubsytem();

	// Creates the Xbox Controllers
	private final CommandXboxController driverController = new CommandXboxController(Constants.OperatorConstants.DRIVER);
	private final CommandXboxController copilotController = new CommandXboxController(Constants.OperatorConstants.COPILOT);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();

		// Configure the auto builder
		configureAutoBuilder();

		DriverStation.silenceJoystickConnectionWarning(true);

		// AUTO COMMANDS
    NamedCommands.registerCommand("Shoot First 8", (shooter.shooterOnCommand()).withTimeout(4));
    NamedCommands.registerCommand("Kicker For Shooting", (kicker.kickerOnCommand()).withTimeout(4));
    NamedCommands.registerCommand("Roller For Shooting", (roller.rollerOnCommand()).withTimeout(4));
    //
    NamedCommands.registerCommand("Reverse Kicker", (kicker.kickerReverseCommand()).withTimeout(.25));
    //
    NamedCommands.registerCommand("Shoot First 8 Forever", shooter.shooterOnCommand());
    NamedCommands.registerCommand("Kicker Forever", kicker.kickerOnCommand());
    NamedCommands.registerCommand("Roller Forever", roller.rollerOnCommand());
    //
    NamedCommands.registerCommand("Intake Fuel", intake.intakeLiftDownCommand().withTimeout(1)
			.andThen(intake.intakeSpinnyCommand()).withTimeout(4));

		// Builds an auto chooser
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	private Command driveDefaultCommand() {
		return new RunCommand(
			() -> driveBase.drive(
				-driverController.getLeftY(),
				-driverController.getLeftX(),
				driverController.getRightX(),
				true,
				driverController.rightTrigger().getAsBoolean()
			),
		driveBase
		);
	}

	private void configureBindings() {

		driveBase.setDefaultCommand(driveDefaultCommand());

		/*
		//// One Controller Set-Up
		driverController.a().onTrue(Commands.runOnce(driveBase::zeroGyro)); // Zeros the gyro
		//
		driverController.leftBumper().onTrue(kicker.kickerOffCommand()); // Turns the kicker off
		driverController.rightBumper().onTrue(roller.rollerReverseCommand().withTimeout(.5).andThen(kicker.kickerOnCommand())); // Turns the kicker on after running it in reverse for .5 seconds to clear it
		//
		driverController.leftTrigger().whileTrue(
			new LockOnToHub(
				driveBase,
				limelightSubsytem,
				() -> -driverController.getLeftY(),
				() -> -driverController.getLeftX(),
				() -> driverController.rightTrigger().getAsBoolean()
			)
		); // Locks the robot onto the hub using the limelight
		//
		// shooter.setDefaultCommand(shooter.shooterCommand(driverController, copilotController)); // Controls the shooter
		// kicker.setDefaultCommand(kicker.kickerCommand(driverController, copilotController)); // Controls the kicker
    // roller.setDefaultCommand(roller.rollerCommand(driverController, copilotController)); // Controls the roller
		intake.setDefaultCommand(intake.oneControllerIntakeCommand(driverController)); // Controls the intake lift motion and the intake spinny
		//
		driverController.povUp().onTrue(roller.rollerOnCommand()).onFalse(roller.rollerCommand(driverController, copilotController)); // Turns the roller on
		driverController.povLeft().whileTrue(Commands.runOnce(driveBase::zeroWheels)); // Zeros the wheels
		driverController.povRight().whileTrue(roller.jiggleRollerCommand()). whileFalse(roller.rollerCommand(driverController, copilotController)); //Jiggles the roller back and forth
		////
		*/

		//// Two Controller Set-Up
		driverController.a().whileTrue(
			new LockOnToHub(
				driveBase,
				limelightSubsytem,
				() -> -driverController.getLeftY(),
				() -> -driverController.getLeftX(),
				() -> driverController.rightTrigger().getAsBoolean()
			)
		); // Locks the robot onto the hub using the limelight
		driverController.b().whileTrue(Commands.runOnce(driveBase::zeroWheels)); // Zeros the wheels
		driverController.x().whileTrue(Commands.runOnce(driveBase::antiPushWheels)); // Puts the wheels in an X pattern to "lock" them
		driverController.y().onTrue(Commands.runOnce(driveBase::zeroGyro)); // Zeros the gyro
		// System.out.println("brad");
		driverController.leftBumper().onTrue(kicker.kickerOffCommand()); // Turns the kicker off
		driverController.rightBumper().onTrue(kicker.kickerReverseCommand().withTimeout(.25).andThen(kicker.kickerOnCommand())); // Turns the kicker on after running it in reverse for .5 seconds to clear it
    driverController.rightBumper().whileTrue(Commands.runOnce(driveBase::antiPushWheels)); // Puts the wheels in an X pattern to "lock" them while shooting
		//
		shooter.setDefaultCommand(shooter.shooterCommand(driverController, copilotController)); // Controls the shooter
		// kicker.setDefaultCommand(kicker.kickerCommand(driverController, copilotController)); // Controls the kicker
		roller.setDefaultCommand(roller.rollerCommand(driverController, copilotController)); // Controls the roller
		///
		//
		copilotController.leftBumper().onTrue(kicker.kickerOffCommand()); // Turns the kicker off
		copilotController.rightBumper().onTrue(kicker.kickerReverseCommand().withTimeout(.25).andThen(kicker.kickerOnCommand())); // Turns the kicker on after running it in reverse for .5 seconds to clear it
    copilotController.rightBumper().whileTrue(Commands.runOnce(driveBase::antiPushWheels)); // Puts the wheels in an X pattern to "lock" them while shooting
		//
		intake.setDefaultCommand(intake.intakeCommand(copilotController)); // Controls the intake lift motion and the intake spinny
		//
		copilotController.povUp().onTrue(roller.rollerOnCommand()).onFalse(roller.rollerCommand(driverController, copilotController)); // Turns the roller on
    copilotController.povLeft().onTrue(Commands.runOnce(driveBase::antiPushWheels)); // Puts the wheels in an X pattern and "locks" them
    copilotController.povRight().whileTrue(roller.jiggleRollerCommand()).onFalse(roller.rollerOffCommand().andThen(roller.rollerCommand(driverController, copilotController))); // Jiggles the roller back and forth
		////
	}

	public final void configureAutoBuilder() {

		// Load the RobotConfig from the GUI settings 
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } 
		catch (IOException | ParseException e) {
      DriverStation.reportError("Failed to load PathPlanner robot config", e.getStackTrace());
  		return;
    }

    // Configure the AutoBuilder
		AutoBuilder.configure(
				driveBase::getPose, // Robot pose supplier
				driveBase::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
				driveBase::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				driveBase::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
				new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
						new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
						new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
				),
				config, // The robot configuration
				() -> {
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE
					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
			driveBase // Reference to which subsystem to set requirements
		);
	}

	private final SendableChooser<Command> autoChooser;

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {

		driveBase.zeroGyro();
		driveBase.resetEncoders();
		driveBase.getOdometry().resetPosition(new Rotation2d(), driveBase.modulePositions(), new Pose2d());

		return autoChooser.getSelected();
	}

	// Resets the heading of the gyroscope
  public void resetGyro() {
    driveBase.zeroGyro();
  }

	// Adds 180 degrees to the gyroscope if we are on red alliance to make the 
	// gyroscope and controls correct so the driver doesn't have to do it manually 
	public void allianceRelativeGyroscopeControl() {
		driveBase.allianceRelativeGyroscopeControl();
	}

	// Match Start Protocol
	public void matchStartProtocol(){
		driveBase.zeroWheels();
    driveBase.resetEncoders();
    driveBase.getOdometry().resetPosition(new Rotation2d(), driveBase.modulePositions(), new Pose2d());
	}
}
