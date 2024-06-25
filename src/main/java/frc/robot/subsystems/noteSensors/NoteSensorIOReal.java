package frc.robot.subsystems.noteSensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class NoteSensorIOReal implements NoteSensorIO {
  /**
   * The through beam sensor for detecting if notes are in the intake.
   */
  private DigitalInput intakeSensor = new DigitalInput(Constants.NoteSensors.intakeSensorDIOPort);
  /**
   * The through beam sensor detecting if the note is in position.
   */
  private DigitalInput noteInPositionSensor = new DigitalInput(Constants.NoteSensors.noteInPositionSensorDIOPort);
  /**
   * The through beam sensor detecting if the note is transitioning to the resting position.
   */
  private DigitalInput noteInTransitionSensor = new DigitalInput(Constants.NoteSensors.noteInTransitionSensorDIOPort);

  @Override
  public void updateInputs(NoteSensorIOInputs inputs) {
    inputs.intakeSensorActivated = intakeSensor.get();
    inputs.noteInPositionSensorActivated = noteInPositionSensor.get();
    inputs.noteInTransitionSensorActivated = noteInTransitionSensor.get();
  }
}
