// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPoses;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.DashboardSubsystem;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ArmPoseCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	private final DriveSubsystem driveSubsystem = new DriveSubsystem();
	private final DashboardSubsystem dashboardSubsystem = new DashboardSubsystem(driveSubsystem);
	private final ArmSubsystem armSubsystem = new ArmSubsystem();

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController driverController = new CommandXboxController(
			OperatorConstants.kDriverControllerPort);
	private final CommandGenericHID operatorController = new CommandGenericHID(1);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		// Schedule `ExampleCommand` when `exampleCondition` changes to `true`
		new Trigger(exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(exampleSubsystem));

		// Schedule `exampleMethodCommand` when the Xbox controller's B button is
		// pressed, cancelling on release.
		driverController.a().onTrue(new ArmPoseCommand(armSubsystem, ArmPoses.MID_SCORE));
		driverController.b().onTrue(new ArmPoseCommand(armSubsystem, ArmPoses.MID_INTAKE));
		driverController.y().onTrue(new ArmPoseCommand(armSubsystem, ArmPoses.LOW_SCORE));
		driverController.x().onTrue(new ArmPoseCommand(armSubsystem, ArmPoses.TUCKED));

		
		new Trigger(driverController.back()).onTrue(driveSubsystem.toggleFieldCentric());
		new Trigger(driverController.start()).onTrue(driveSubsystem.zeroHeading());

		// Swerve Drive method is set as default for drive subsystem
		driveSubsystem.setDefaultCommand(
				new RunCommand(
						() -> driveSubsystem.drive(
								driverController.getLeftY() * DriveConstants.kMaxSpeedMetersPerSecond, // x axis
								driverController.getLeftX() * DriveConstants.kMaxSpeedMetersPerSecond, // y axis
								driverController.getRightX() * DriveConstants.kMaxRPM // z axis
						),
						driveSubsystem));

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return Autos.exampleAuto(exampleSubsystem);
	}
}
