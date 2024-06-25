package frc.robot.subsystems.noteSensors;

import org.littletonrobotics.junction.AutoLog;

public interface NoteSensorIO {
  @AutoLog
  public static class NoteSensorIOInputs {
    /**
     * If the through beam sensor for detecting if notes are in the intake is activated
     * (meaning the beam is broken).
     */
    public boolean intakeSensorActivated = false;
    /**
     * If the through beam sensor detecting if the note is transitioning to the resting position is activated
     * (meaning the beam is broken).
     */
    public boolean noteInTransitionSensorActivated = false;
    /**
     * If the through beam sensor detecting if the note is in position is activated
     * (meaning the beam is broken).
     */
    public boolean noteInPositionSensorActivated = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(NoteSensorIOInputs inputs) {}
}