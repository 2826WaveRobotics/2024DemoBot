package frc.robot.subsystems.lighting;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.NoteState;
import frc.robot.subsystems.drive.Swerve;

public class Lighting extends SubsystemBase {
    private static Lighting instance = null;
    public static Lighting getInstance() {
        if (instance == null) {
            switch(Constants.currentMode) {
                case REAL:
                    instance = new Lighting(new LightingIOReal());
                default:
                    instance = new Lighting(new LightingIO() {});
            }
        }
        return instance;
    }

    private LightingIO lightingIO;

    private Lighting(LightingIO io) {
        this.lightingIO = io;
    }

    /**
     * The possible lighting states.  
     * This enum MUST match the Arduino's enum definitions for the light states.
     */
    public enum LightState {
        preStartState,
        autoState,
        teleopNoteIntookState,
        teleopTransportState,
        teleopNoteReadyState,
        teleopEjectingNoteState,
        teleopStaticState
    }

    /**
     * Gets the current lighting state.
     * @return
     */
    private LightState getLightingState() {
        if(DriverStation.isDisabled()) return LightState.preStartState;
        if(DriverStation.isAutonomous()) return LightState.autoState;

        NoteState noteManagementState = Superstructure.getInstance().getNoteState();

        // Teleop states
        if(noteManagementState == NoteState.IntakingNote)  return LightState.teleopNoteIntookState;
        if(noteManagementState == NoteState.MovingNote)    return LightState.teleopTransportState;
        if(noteManagementState == NoteState.ReadyToLaunch) return LightState.teleopNoteReadyState;
        if(noteManagementState == NoteState.EjectingNote)  return LightState.teleopEjectingNoteState;

        return LightState.teleopStaticState;
    }

    int cycle = 0;

    /**
     * Updates the arduino with the state it requires.
     * This is run periodically, no matter if the robot is enabled, by the scheduler.
     */
    @Override
    public void periodic() {
        cycle++;
        if(cycle > 3) {
            cycle = 0;
            LightState lightState = getLightingState();
            double robotSpeed = Swerve.getInstance().getRobotSpeed();
            Logger.recordOutput("Lighting/lightState", lightState.toString());
            Logger.recordOutput("Lighting/robotSpeed", robotSpeed);
            lightingIO.setLightState(robotSpeed, lightState);
        }
    }
}
