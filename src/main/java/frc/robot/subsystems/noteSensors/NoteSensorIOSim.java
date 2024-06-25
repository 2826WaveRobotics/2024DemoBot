package frc.robot.subsystems.noteSensors;

public class NoteSensorIOSim implements NoteSensorIO {
  @Override
  public void updateInputs(NoteSensorIOInputs inputs) {
    inputs.intakeSensorActivated = false;
    inputs.noteInPositionSensorActivated = false;
    inputs.noteInTransitionSensorActivated = false;
  }
}
