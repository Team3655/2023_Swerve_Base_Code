// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmSwitchCommand extends SequentialCommandGroup {

	private static ArmSubsystem armSubsystem;
	// private static LimelightSubsystem limelightSubsystem;

	/** Creates a new ArmSwitchCommand. */
	public ArmSwitchCommand() {

		armSubsystem = RobotContainer.armSubsystem;
		//limelightSubsystem = RobotContainer.limelightSubsystem;

		// ArmPoses prevArmPose = armSubsystem.getArmState();
		System.out.println(armSubsystem.getArmState());

		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new ArmPoseCommand(ArmPoses.TUCKED),
				new PrintCommand("SWITCHING SIDES!!!!!!!!!!!!!!!!!"),
				new InstantCommand(() -> armSubsystem.ToggleSide()));
	}
}
