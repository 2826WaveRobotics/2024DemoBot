package frc.robot.subsystems.noteSensors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NoteSensors extends SubsystemBase {
  private static NoteSensors instance = null;
  public static NoteSensors getInstance() {
    if (instance == null) {
        switch (Constants.currentMode) {
          case REAL:
            instance = new NoteSensors(new NoteSensorIOReal());
            return instance;
          case REPLAY:
            instance = new NoteSensors(new NoteSensorIO() {});
            return instance;
          case SIM:
            instance = new NoteSensors(new NoteSensorIOSim());
            return instance;
        }
    }
    return instance;
  }

  private final NoteSensorIO noteSensorIO;
  private final NoteSensorIOInputsAutoLogged inputs = new NoteSensorIOInputsAutoLogged();

  private NoteSensors(NoteSensorIO noteSensorIO) {
    this.noteSensorIO = noteSensorIO;

    if(!Constants.enableNonEssentialShuffleboard) return;
    ShuffleboardTab notesTab = Shuffleboard.getTab("Notes");
    // notesTab.addBoolean("Intake sensor activated", () -> inputs.intakeSensorActivated);
    notesTab.addBoolean("Intake sensor activated", () -> false);
    notesTab.addBoolean("Note in position sensor activated", () -> inputs.noteInPositionSensorActivated);
    notesTab.addBoolean("Note in transition sensor activated", () -> inputs.noteInTransitionSensorActivated);
  }

  /**
   * Updates the read sensor values.  
   * We don't do this in periodic because we want to synchronize the sensor reads with the superstructure state updates to avoid extra latency.
   */
  public void updateSensorValues() {
    noteSensorIO.updateInputs(inputs);
    Logger.processInputs("NoteSensors", inputs);
  }

  /**
   * Gets if the through beam sensor for detecting if notes are in the intake is activated (meaning there's a note there).
   * @return
   */
  public boolean getIntakeSensorActivated() {
    // return inputs.intakeSensorActivated;
    // Temporary
    return false;
  }

  /**
   * Gets if the through beam sensor for detecting if the note is in position is activated (meaning there's a note there).
   * @return
   */
  public boolean getNoteInPositionSensorActivated() {
    return inputs.noteInPositionSensorActivated;
  }

  /**
   * Gets if the through beam sensor for detecting if the note is transitioning to the resting position is activated
   * (meaning there's a note there).
   * @return
   */
  public boolean getNoteInTransitionSensorActivated() {
    return inputs.noteInTransitionSensorActivated;
  }
}
