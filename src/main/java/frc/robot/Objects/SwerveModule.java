// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Objects;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
	/** Creates a new SwerveModule. */

	private final CANSparkMax driveMotor;
	private final CANSparkMax turningMotor;

	private final CANCoder absoluteEncoder;
	private final RelativeEncoder angularEncoder;
	private final RelativeEncoder driveEncoder;

	private final SparkMaxPIDController angularPID;
	private final SparkMaxPIDController drivePID;

	private final double angleZero;

	private final String moduleName;

	public SwerveModule(
			String moduleName,
			int driveMotorChannel,
			int turningMotorChannel,
			int turningEncoderPort,
			double angleZero,
			double[] angularPID,
			double[] drivePID) {

		this.moduleName = moduleName;

		// Initialize the motors
		this.angleZero = angleZero;

		driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
		turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

		turningMotor.restoreFactoryDefaults();
		driveMotor.restoreFactoryDefaults();

		// Initalize CANcoder
		absoluteEncoder = new CANCoder(turningEncoderPort);

		absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
		absoluteEncoder.configMagnetOffset(0);
		absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100);

		// Initialize Spark MAX encoders
		angularEncoder = turningMotor.getEncoder();

		turningMotor.setInverted(true);

		angularEncoder.setPositionConversionFactor(ModuleConstants.kturnGearRatio * 2d * Math.PI); // radians
		angularEncoder.setVelocityConversionFactor(
				ModuleConstants.kturnGearRatio
						* 2d * Math.PI
						* (1d / 60d)); // radians per second

		// m_angularEncoder.setPosition(Math.toRadians(m_absoluteEncoder.getAbsolutePosition())
		// - Math.toRadians(angleZero));

		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(ModuleConstants.kdriveGearRatio * 2d * Math.PI); // meters
		driveMotor.getEncoder().setVelocityConversionFactor(
				ModuleConstants.kdriveGearRatio
						* ModuleConstants.kwheelCircumference
						* (1d / 60d)); // meters per second

		// Initialize PID's
		this.angularPID = turningMotor.getPIDController();
		this.angularPID.setP(angularPID[0]);
		this.angularPID.setI(angularPID[1]);
		this.angularPID.setD(angularPID[2]);

		this.drivePID = driveMotor.getPIDController();
		this.drivePID.setP(drivePID[0]);
		this.drivePID.setI(drivePID[1]);
		this.drivePID.setD(drivePID[2]);

		this.angularPID.setFF(ModuleConstants.kTurnFeedForward);
		this.drivePID.setFF(ModuleConstants.kDriveFeedForward);

		this.angularPID.setFeedbackDevice(turningMotor.getEncoder());
		this.drivePID.setFeedbackDevice(driveMotor.getEncoder());

		this.angularPID.setPositionPIDWrappingEnabled(true);
		this.angularPID.setPositionPIDWrappingMinInput(Math.toRadians(0));
		this.angularPID.setPositionPIDWrappingMaxInput(Math.toRadians(360));

		this.angularPID.setOutputRange(-1, 1);
		this.drivePID.setOutputRange(-1, 1);

		// Configure current limits for motors - prevents disabling/brownouts
		driveMotor.setIdleMode(IdleMode.kCoast);
		turningMotor.setIdleMode(IdleMode.kCoast);
		turningMotor.setSmartCurrentLimit(30);
		driveMotor.setSmartCurrentLimit(30);

		resetAngleToAbsolute();

		SmartDashboard.putNumber(this.moduleName + " Init Spark Enc", resetAngleToAbsolute());

	}

	public double resetAngleToAbsolute() {
		double angle = Math.toRadians(absoluteEncoder.getAbsolutePosition() - this.angleZero);
		SmartDashboard.putString(moduleName + " Abs. Error", absoluteEncoder.getLastError().toString());
		angularEncoder.setPosition(angle);

		return this.angularEncoder.getPosition();
	}

	// Returns headings of the module
	public double getAbsoluteHeading() {
		return absoluteEncoder.getAbsolutePosition();
	}

	public double getRelativeHeading() {
		return Math.toDegrees(angularEncoder.getPosition());
	}

	// Returns current position of the modules
	public SwerveModulePosition getPosition() {

		double m_moduleAngleRadians = angularEncoder.getPosition();

		double m_distanceMeters = driveEncoder.getPosition();

		return new SwerveModulePosition(m_distanceMeters, new Rotation2d(m_moduleAngleRadians));
	}

	public void setDesiredState(SwerveModuleState desiredState) {

		double m_moduleAngleRadians = angularEncoder.getPosition();
		// Optimize the reference state to avoid spinning further than 90 degrees to
		// desired state
		SwerveModuleState optimizedState = SwerveModuleState.optimize(
				desiredState,
				new Rotation2d(m_moduleAngleRadians));

		angularPID.setReference(
				optimizedState.angle.getRadians(),
				ControlType.kPosition);

		drivePID.setReference(
				optimizedState.speedMetersPerSecond,
				ControlType.kVelocity);

	}

	public void resetEncoders() {
		angularEncoder.setPosition(0);
		driveMotor.getEncoder().setPosition(0);
	}

	public void putConversionFactors() {
		SmartDashboard.putNumber(moduleName + " A P Conversion", angularEncoder.getPositionConversionFactor());
		SmartDashboard.putNumber(moduleName + " A V Conversion", angularEncoder.getVelocityConversionFactor());
		SmartDashboard.putNumber(moduleName + " D V Conversion", driveEncoder.getVelocityConversionFactor());
	}

	@Override
	public void periodic() {

		// This method will be called once per scheduler run

	}
}
