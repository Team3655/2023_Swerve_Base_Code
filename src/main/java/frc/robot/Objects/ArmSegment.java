package frc.robot.Objects;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSegment {

	// region properties

	/** Motor controllers for the segment */
	private CANSparkMax rightMotor, leftMotor;

	/** PID controllers for the segment */
	public SparkMaxPIDController rightPIDController, leftPIDController;

	/** Encoders for the segment */
	private RelativeEncoder rightEncoder, leftEncoder;

	/** Real and target Angles for the arms */
	private double targetTheta;

	/** the angle constraints on the arm */
	private double minTheta, maxTheta;

	/** Controls the direction of the arm */
	private int targetSign;

	// endregion

	public ArmSegment(int rightPort, int leftPort, double gearRatio, Boolean invertLeft) {

		targetSign = 1;

		// region def_motors
		// creates left and right arm motors
		rightMotor = new CANSparkMax(leftPort, MotorType.kBrushless);
		leftMotor = new CANSparkMax(rightPort, MotorType.kBrushless);

		/**
		 * The restoreFactoryDefaults method can be used to reset the configuration
		 * parameters in the SPARK MAX to their factory default state. If no argument is
		 * passed, these parameters will not persist between power cycles
		 */
		rightMotor.restoreFactoryDefaults();
		leftMotor.restoreFactoryDefaults();

		// sets motor defaults to break
		rightMotor.setIdleMode(IdleMode.kCoast);
		leftMotor.setIdleMode(IdleMode.kCoast);

		/**
		 * In order to use PID functionality for a controller, a SparkMaxPIDController
		 * object is constructed by calling the getPIDController() method on an existing
		 * CANSparkMax object
		 */
		rightPIDController = rightMotor.getPIDController();
		leftPIDController = leftMotor.getPIDController();

		// Encoder object created to display position values
		rightEncoder = rightMotor.getEncoder();
		leftEncoder = leftMotor.getEncoder();

		// Tells the motors to automatically convert degrees to rotations
		rightEncoder.setPositionConversionFactor((2 * Math.PI) / gearRatio);
		leftEncoder.setPositionConversionFactor((2 * Math.PI) / gearRatio);

		// Sets the left motor to be inverted if it needs to be
		if (invertLeft) {
			leftMotor.setInverted(true);
		} else {
			rightMotor.setInverted(true);
		}

		// endregion
	}

	// region: setters

	/** Sets the pid referance point to the target theta of the segment */
	public void setReference() {
		rightPIDController.setReference(targetTheta * targetSign, CANSparkMax.ControlType.kPosition);
		leftPIDController.setReference(targetTheta * targetSign, CANSparkMax.ControlType.kPosition);
	}

	public void invertMotors() {
		rightMotor.setInverted(!rightMotor.getInverted());
		leftMotor.setInverted(!leftMotor.getInverted());
	}

	/**
	 * Sets the sign that controls the dominant side of the robot
	 * 
	 * @param sign the Sign to set
	 */
	public void setSign(int sign) {
		targetSign = sign;
	}

	/** sets the angle constraint and the speed constraint */
	public void setConstraints(double theta, double maxOutput) {
		maxTheta = theta;
		minTheta = -theta;
		rightPIDController.setOutputRange(-maxOutput, maxOutput);
		leftPIDController.setOutputRange(-maxOutput, maxOutput);
	}

	/**
	 * Forces a given angle to be between thw min and max constraints
	 * 
	 * @param theta the angle to be constrained
	 * @return the limited value of theta
	 */
	public double constrain(double theta) {

		if (theta >= maxTheta) {
			return maxTheta;

		} else if (theta <= minTheta) {
			return minTheta;
		}

		return theta;
	}

	/**
	 * Sets the target angle of the arm IN RADIANS!
	 * 
	 * @param theta the target angle to be set
	 */
	public void setTargetTheta(double theta) {
		theta = constrain(theta);
		targetTheta = Math.toRadians(theta);
	}

	/**
	 * sets the PID values for the arm segment
	 * 
	 * @param P Just look up a PID loop
	 * @param I Seriously hurry up!
	 * @param D Don't make me wait on behalf of your ignorance
	 */
	public void setPID(double P, double I, double D) {
		rightPIDController.setP(P);
		rightPIDController.setI(I);
		rightPIDController.setD(D);
		leftPIDController.setP(P);
		leftPIDController.setI(I);
		leftPIDController.setD(D);
	}

	/**
	 * Used to set the maximum power draw of the arm
	 * 
	 * @param limit the maximum allowed power draw
	 */
	public void setSmartCurrentLimit(int limit) {
		rightMotor.setSmartCurrentLimit(limit);
		leftMotor.setSmartCurrentLimit(limit);
	}

	// endregion

	// region: getters

	/**
	 * @return the m_targetTheta
	 */
	public double getTargetTheta() {
		return targetTheta;
	}

	/** Returns the actual angle of the real arm (not the same as the target) */
	public double getRealTheta() {
		return Math.toDegrees((rightEncoder.getPosition() + leftEncoder.getPosition()) / 2);
	}

	/**
	 * Checks of the arm is at its target position
	 * 
	 * @param deadBand the number of degrees of error that is acceptable
	 * @return True if the Real angle is within the deadband of the target
	 */
	public boolean getAtTarget(double deadBand) {
		// get absolute value of the difference
		double error = Math.abs(getRealTheta() - targetTheta);
		// convert deadband to radians
		deadBand = Math.toRadians(deadBand);

		if (error < deadBand) {
			return true;
		}
		return false;
	}

	// endregion

}
