package frc.robot.Utils;

import frc.robot.Constants.OperatorConstants;

public class JoystickUtils {
	
	public static double deadBand(double x) {
		if (Math.abs(x) < OperatorConstants.KDeadBand) {
			return 0;
		}
		return x;
	}

}
