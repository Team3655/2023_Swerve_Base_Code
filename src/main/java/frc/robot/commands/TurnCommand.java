// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;

/** A command that will turn the robot to the specified angle. */
public class TurnCommand extends PIDCommand {
	
	/**
	 * Turns to robot to the specified angle.
	 *
	 * @param targetAngleDegrees The angle to turn to
	 * @param driveSubsystem     The drive subsystem to use
	 */
	public TurnCommand(double targetAngleDegrees, DriveSubsystem driveSubsystem) {
		super(
				new PIDController(
						DriveConstants.kTurnP,
						DriveConstants.kTurnI,
						DriveConstants.kTurnD),
				// Close loop on heading
				driveSubsystem::getHeading,
				// Set reference to target
				targetAngleDegrees,
				// Pipe output to turn robot
				output -> driveSubsystem.drive(0, 0, output), driveSubsystem);

		// Set the controller to be continuous (because it is an angle controller)
		getController().enableContinuousInput(-180, 180);
		// Set the controller tolerance - the delta tolerance ensures the robot is
		// stationary at the
		// setpoint before it is considered as having reached the reference
		getController().setTolerance(
				DriveConstants.kTurnToleranceDeg,
				DriveConstants.kTurnRateToleranceDegPerS);
	}

	@Override
	public boolean isFinished() {
		// End when the controller is at the reference.
		return getController().atSetpoint();
	}
}