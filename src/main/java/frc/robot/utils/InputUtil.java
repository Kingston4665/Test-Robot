package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OperatorConstants;

public class InputUtil {
  /**
   * Deadbands a joystick value
   * 
   * @param value the joystick value to deadband
   * @return the deadbanded joystick value
   */
  public static double deadband(double value) {
    // Rescale above the threshold so the first nonzero joystick value starts
    // at zero instead of abruptly jumping to ten percent motor demand.
    return MathUtil.applyDeadband(value, OperatorConstants.DEADBAND);
  }
}
