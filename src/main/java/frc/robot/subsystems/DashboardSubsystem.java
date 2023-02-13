// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DashboardSubsystem extends SubsystemBase {

	//private static DriveSubsystem m_driveSubsystem;
	private final DriveSubsystem driveSubsystem;

	public DashboardSubsystem(DriveSubsystem subsystem) {
		driveSubsystem = subsystem;
	}

	@Override
	public void periodic() {
		ModuleTelemetry();
	}

	public static class PIDConstants {

		// PID Shuffleboard tab
		private static ShuffleboardTab PIDTab = Shuffleboard.getTab("PID");

		// Entries of PathPlanner PID Values
		private static GenericEntry PathPlanner_X_kP = PIDTab.addPersistent("PathPlanner X kP", 0).getEntry();

		private static GenericEntry PathPlanner_X_kd = PIDTab.addPersistent("PathPlanner X kD", 0).getEntry();

		private static GenericEntry PathPlanner_Y_kP = PIDTab.addPersistent("PathPlanner Y kP", 0).getEntry();

		private static GenericEntry PathPLanner_Y_kD = PIDTab.addPersistent("PathPLanner Y kD", 0).getEntry();

		private static GenericEntry PathPlanner_rot_kP = PIDTab.addPersistent("PathPlanner Rot kP", 0).getEntry();

		private static GenericEntry PathPlanner_rot_kD = PIDTab.addPersistent("PathPlanner Rot kD", 0).getEntry();

		public static double getPlanner_X_kP() {
			return PathPlanner_X_kP.getDouble(0);
		}

		public static double getPlanner_X_kD() {
			return PathPlanner_X_kd.getDouble(0);
		}

		public static double getPlanner_Y_kP() {
			return PathPlanner_Y_kP.getDouble(0);
		}

		public static double getPlanner_Y_kD() {
			return PathPLanner_Y_kD.getDouble(0);
		}

		public static double getPlanner_Rot_kP() {
			return PathPlanner_rot_kP.getDouble(0);
		}

		public static double getPlanner_Rot_kD() {
			return PathPlanner_rot_kD.getDouble(0);
		}

		// Entries for Drive/Turn PID

		private static GenericEntry Drive_kP = PIDTab.addPersistent("Drive kP", 0).getEntry();

		private static GenericEntry Drive_kD = PIDTab.addPersistent("Drive kD", 0).getEntry();

		private static GenericEntry Turn_kP = PIDTab.addPersistent("Turn kP", 0).getEntry();

		private static GenericEntry Turn_kD = PIDTab.addPersistent("Turn kD", 0).getEntry();

		public static double getDrive_kP() {
			return Drive_kP.getDouble(0);
		}

		public static double getDrive_kD() {
			return Drive_kD.getDouble(0);
		}

		public static double getTurn_kP() {
			return Turn_kP.getDouble(0);
		}

		public static double getTurn_kD() {
			return Turn_kD.getDouble(0);
		}

		// Entries for Major Arm PID

		private static GenericEntry MajorArm_kP = PIDTab.addPersistent("Major Arm kP", 0).getEntry();

		private static GenericEntry MajorArm_kD = PIDTab.addPersistent("Major Arm kD", 0).getEntry();

		private static GenericEntry MinorArm_kP = PIDTab.addPersistent("Minor Arm kP", 0).getEntry();

		private static GenericEntry MinorArm_kD = PIDTab.addPersistent("Minor Arm kD", 0).getEntry();

		public static double getMajor_kP() {
			return MajorArm_kP.getDouble(0);
		}

		public static double getMajor_kD() {
			return MajorArm_kD.getDouble(0);
		}

		public static double getMinor_kP() {
			return MinorArm_kP.getDouble(0);
		}

		public static double getMinor_kD() {
			return MinorArm_kD.getDouble(0);
		}
	}

	
	private ShuffleboardTab m_ModuleTab = Shuffleboard.getTab("Modules");


	private GenericEntry gyro = m_ModuleTab.add("Gyro", 0).getEntry();

	public void ModuleTelemetry() {
		// m_FrontLeftHeading.setDouble(m_drive.getFrontLeftHeading());
		// m_RearLeftHeading.setDouble(m_drive.getRearLeftHeading());
		// m_FrontRightHeading.setDouble(m_drive.getFrontRightHeading());
		// m_RearRightHeading.setDouble(m_drive.getRearRightHeading());

		gyro.setDouble(driveSubsystem.getHeading());
	}



	
}
