// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmSwitchCommand extends InstantCommand {

	ArmSubsystem armSubsystem;
	LimelightSubsystem limelightSubsystem;

  public ArmSwitchCommand(ArmSubsystem armSubsystem, LimelightSubsystem limelightSubsystem) {

	this.armSubsystem = armSubsystem;
	this.limelightSubsystem = limelightSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
	addRequirements(armSubsystem);
	addRequirements(limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
	armSubsystem.ToggleSide();
	
  }

}
