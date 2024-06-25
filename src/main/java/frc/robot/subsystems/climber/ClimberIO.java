package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double leftClimberCurrentDrawAmps = 0;
    public double rightClimberCurrentDrawAmps = 0;

    public double leftClimberSpeedRPM = 0;
    public double rightClimberSpeedRPM = 0;

    public double leftPosition = 0;
    public double rightPosition = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}
  
  /** Resets the encoder value for the left motor to 0. */
  public default void resetLeftEncoder() {}
  
  /** Resets the encoder value for the right motor to 0. */
  public default void resetRightEncoder() {}

  /**
   * Sets the left motor speed in RPM. Positive values move the elevator downward.
   * @param speed The target speed, in RPM.
   */
  public default void setLeftSpeed(double speed) {}
  /**
   * Sets the right motor speed in RPM. Positive values move the elevator downward.
   * @param speed
   */
  public default void setRightSpeed(double speed) {}
  /**
   * Sets the right motor position in rotations, where 0 is the resting position and positive numbers are upward.
   * @param position
   */
  public default void setRightPosition(double position) {}
  /**
   * Sets the left motor position in rotations, where 0 is the resting position and positive numbers are upward.
   * @param position
   */
  public default void setLeftPosition(double position) {}

  /**
   * Sets the current limits on both climber motors.  
   * Used because we have a lower current limit when at the top of the climbing cycle than when moving.
  * @param smartLimit The smart current limit for the motors.
   */
  public default void useCurrentLimit(int smartLimit) {}
}