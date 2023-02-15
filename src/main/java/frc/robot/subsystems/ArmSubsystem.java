package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;
import frc.robot.Objects.ArmSegment;

public class ArmSubsystem extends SubsystemBase {

	// region properties

	/** used for controling the height of the arm */
	public enum ArmPoses {
		TUCKED,
		LOW_SCORE,
		MID_SCORE,
		HIGH_SCORE,
		LOW_INTAKE,
		MID_INTAKE,
		HIGH_INTAKE,
		DRIVER_CONTROL
	}

	/** controls the side of the robot the arm is on */
	private boolean isFront;

	/** the variable setting the height of the arm */
	ArmPoses armState;

	// create arms
	public ArmSegment majorArm;
	public ArmSegment minorArm;

	// endregion

	public ArmSubsystem() {
		// region: def arms

		// major arm defs
		majorArm = new ArmSegment(
				ArmConstants.kRightMajorArmPort,
				ArmConstants.kLeftMajorArmPort,
				ArmConstants.kMajorArmTicks,
				true);

		majorArm.setPID(
				ArmConstants.kMajorArmP,
				ArmConstants.kMajorArmI,
				ArmConstants.kMajorArmD);

		majorArm.setConstraints(
				ArmConstants.kMajorArmConstraints,
				ArmConstants.kMajorArmPIDOutputLimit);

		// minor arm defs
		minorArm = new ArmSegment(
				ArmConstants.kRightMinorArmPort,
				ArmConstants.kLeftMinorArmPort,
				ArmConstants.kMinorArmTicks,
				false);

		minorArm.setPID(
				ArmConstants.kMinorArmP,
				ArmConstants.kMinorArmI,
				ArmConstants.kMinorArmD);

		minorArm.setConstraints(
				ArmConstants.kMinorArmConstraints,
				ArmConstants.kMinorArmPIDOutputLimit);
		// endregion

		// the default state of the arms
		isFront = true;

		setArmState(ArmPoses.TUCKED);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		SmartDashboard.putNumber("major real theta: ", majorArm.getRealTheta());
		SmartDashboard.putNumber("minor real theta: ", minorArm.getRealTheta());

		// Address the arm motors
		majorArm.setReference();
		minorArm.setReference();

	}

	// region setters

	/**
	 * Sets the height of the arm
	 * 
	 * @param pose can be (LOW_SCORE, MID_SCORE, HIGH_SCORE,
	 *             LOW_INTAKE, MID_INTAKE, HIGH_INTAKE)
	 */
	public void setArmState(ArmPoses pose) {

		armState = pose;

		switch (armState) {
			// When the arms are tucked in the center of the robot
			// (this is the only legalstarting position)
			case TUCKED:
				majorArm.setTargetTheta(0);
				minorArm.setTargetTheta(0);
				break;

			// Used for scoring in the lowest "hybrid" node
			case LOW_SCORE:
				majorArm.setTargetTheta(45);
				minorArm.setTargetTheta(90);
				break;

			// Used for scoring in the middle node
			case MID_SCORE:
				majorArm.setTargetTheta(75);
				minorArm.setTargetTheta(90);
				break;

			// Used for scoring in the highest node
			case HIGH_SCORE:
				majorArm.setTargetTheta(90);
				minorArm.setTargetTheta(90);
				break;

			// Used for intaking off of the floor
			case LOW_INTAKE:
				majorArm.setTargetTheta(10);
				minorArm.setTargetTheta(100);
				break;

			// Used for intaking from the human player chute
			case MID_INTAKE:
				majorArm.setTargetTheta(30);
				minorArm.setTargetTheta(45);
				break;

			// Used for intaking from the sliding human player station
			case HIGH_INTAKE:
				majorArm.setTargetTheta(80);
				minorArm.setTargetTheta(80);
				break;

			// goes to the pair of angles defined my the TSB driver
			case DRIVER_CONTROL:
				break;
		}

	}

	/** Toggles the dominant side of the robot */
	public CommandBase ToggleSide() {
		return runOnce(() -> {
			isFront = !isFront;
			majorArm.setSign((isFront) ? 1 : -1);
			minorArm.setSign((isFront) ? 1 : -1);
		});
	}

	// endregion

	// region getters

	/**
	 * Used to get the target height of the arm as an enum
	 * 
	 * @return armState: can be (LOW_SCORE, MID_SCORE, HIGH_SCORE, LOW_INTAKE,
	 *         MID_INTAKE, HIGH_INTAKE)
	 */
	public ArmPoses getArmState() {
		return armState;
	}

	/** ruturns true if the target dominant side of the robot is front */
	public boolean getIsFront() {
		return isFront;
	}

	public boolean getAtTarget(double deadBand) {
		if (majorArm.getAtTarget(deadBand) && minorArm.getAtTarget(deadBand)) {
			return true;
		}
		return false;
	}

	// endregion

}
