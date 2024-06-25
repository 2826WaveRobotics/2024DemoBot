package frc.robot.subsystems.transport;

import java.util.HashSet;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The transport subsystem.  This subsystem controls the transport of notes from the intake to the shooter.
 * The transport is a state machine with predefined transitions.
 * Higher priority states can't transition to lower priority states;
 * for example, you can't transition from MovingNote to IntakingNote or EjectingNote to MovingNote.
 */
public class Transport extends SubsystemBase {
  private static Transport instance = null;
  public static Transport getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          instance = new Transport(new TransportIOReal());
          return instance;
        default:
          instance = new Transport(new TransportIO() {});
          return instance;
      }
    }
    return instance;
  }

  private TransportIO transportIO;

  private Transport(TransportIO transportIO) {
    this.transportIO = transportIO;

    if(!Constants.enableNonEssentialShuffleboard) return;
    Shuffleboard.getTab("Notes").addString("Transport state", () -> transportState.toString());
  }

  /**
   * A state the transport can be in.  
   * The transport is a state machine with predefined transitions.
   * Higher priority states can't transition to lower priority states;
   * for example, you can't transition from MovingNote to IntakingNote or EjectingNote to MovingNote.
   */
  public enum TransportState {
    IntakingNote,
    MovingNote,
    EjectingNote,
    OperatorOverride,
    LaunchingNote,
    /**
     * Used when we do a "center sweep" to put notes on our side in auto.
     * Different from IntakingNote because there are no normally-run transitions from this state,
     * so it stays on until the end of auto or we request a transition to "stopped".
     */
    SweepTransport,
    Stopped
  }

  /**
   * The current state of the transport.
   * The transport is a state machine with predefined transitions.
   * Higher priority states can't transition to lower priority states;
   * for example, you can't transition from MovingNote to IntakingNote or EjectingNote to MovingNote.
   */
  private TransportState transportState = TransportState.Stopped;

  /**
   * The operator can override the transport to run at a specific speed.  
   * This is the speed that the transport will run at when being overriden by an operator, in meters per second.
   */
  private double operatorOverrideSpeedMetersPerSecond = 0;
  /**
   * Sets the speed that the transport will run at when being overriden by an operator, in meters per second.
   * @param speedMetersPerSecond
   */
  public void setOperatorOverrideSpeedMetersPerSecond(double speedMetersPerSecond) {
    operatorOverrideSpeedMetersPerSecond = speedMetersPerSecond;
  }

  private class TransportStatePair {
    // Allow unused since this is used for a set of valid state transitions so we only need the data for equality checks
    @SuppressWarnings("unused")
    private TransportState from;
    @SuppressWarnings("unused")
    private TransportState to;

    public TransportStatePair(TransportState from, TransportState to) {
      this.from = from;
      this.to = to;
    }

    @Override
    public boolean equals(Object obj) {
      if (obj instanceof TransportStatePair) {
        TransportStatePair other = (TransportStatePair) obj;
        return other.from == from && other.to == to;
      }
      return false;
  }

    @Override
    public int hashCode() {
      return from.hashCode() + to.hashCode();
    }
  }
    
  /** Valid state transitions, in the order (from, to) */
  private HashSet<TransportStatePair> validStateTransitions = getValidStateTransitions();
  private HashSet<TransportStatePair> getValidStateTransitions() {
    HashSet<TransportStatePair> transitions = new HashSet<>();

    // Standard note path transitions
    transitions.add(new TransportStatePair(TransportState.Stopped, TransportState.IntakingNote));
    transitions.add(new TransportStatePair(TransportState.IntakingNote, TransportState.Stopped));
    transitions.add(new TransportStatePair(TransportState.IntakingNote, TransportState.MovingNote));
    transitions.add(new TransportStatePair(TransportState.MovingNote, TransportState.Stopped));
    transitions.add(new TransportStatePair(TransportState.Stopped, TransportState.LaunchingNote));
    transitions.add(new TransportStatePair(TransportState.LaunchingNote, TransportState.Stopped));

    transitions.add(new TransportStatePair(TransportState.MovingNote, TransportState.LaunchingNote));

    // Operator override transitions
    transitions.add(new TransportStatePair(TransportState.Stopped, TransportState.OperatorOverride));
    transitions.add(new TransportStatePair(TransportState.EjectingNote, TransportState.OperatorOverride));
    transitions.add(new TransportStatePair(TransportState.IntakingNote, TransportState.OperatorOverride));
    transitions.add(new TransportStatePair(TransportState.LaunchingNote, TransportState.OperatorOverride));
    transitions.add(new TransportStatePair(TransportState.MovingNote, TransportState.OperatorOverride));
    transitions.add(new TransportStatePair(TransportState.OperatorOverride, TransportState.Stopped));
    transitions.add(new TransportStatePair(TransportState.SweepTransport, TransportState.OperatorOverride));

    // Ejecting note transitions
    transitions.add(new TransportStatePair(TransportState.IntakingNote, TransportState.EjectingNote));
    transitions.add(new TransportStatePair(TransportState.MovingNote, TransportState.EjectingNote));
    transitions.add(new TransportStatePair(TransportState.Stopped, TransportState.EjectingNote));
    transitions.add(new TransportStatePair(TransportState.LaunchingNote, TransportState.EjectingNote));
    transitions.add(new TransportStatePair(TransportState.EjectingNote, TransportState.Stopped));
    transitions.add(new TransportStatePair(TransportState.Stopped, TransportState.MovingNote));

    // Sweep transport transitions
    transitions.add(new TransportStatePair(TransportState.Stopped, TransportState.SweepTransport));
    transitions.add(new TransportStatePair(TransportState.SweepTransport, TransportState.Stopped));

    return transitions;
  }

  /**
   * Attempts to transition to a new state.  
   * Some state transitions don't work. For example, you can't transition from MovingNote to IntakingNote or EjectingNote to MovingNote.
   * @param newState
   */
  public void attemptTransitionToState(TransportState newState) {
    if (validStateTransitions.contains(new TransportStatePair(transportState, newState))) {
      transportState = newState;
    }
  }

  /**
   * Gets the current state of the transport.
   * @return
   */
  public TransportState getCurrentState() {
    return transportState;
  }

  @Override
  public void periodic() {
    // Speeds are how fast the belt/edge of intake wheels will move at, in meters per second.
    // This is effectively the speed that the note moves.  

    immediatelyUpdateSpeeds();
  }

  /**
   * Resets all transport state; called at the start of auto and teleop.
   */
  public void resetState() {
    transportState = TransportState.Stopped;
  }

  /** Immediately updates the transport motors with the new speeds. Used to reduce latency. */
  public void immediatelyUpdateSpeeds() {
    if (transportState == TransportState.OperatorOverride) {
      transportIO.setTransportSpeed(operatorOverrideSpeedMetersPerSecond);
    } else {
      switch(transportState) {
        case IntakingNote:
          transportIO.setTransportSpeed(Constants.Transport.intakeSpeed);
          break;
        case MovingNote:
          transportIO.setTransportSpeed(Constants.Transport.intakeSpeed * 0.8);
          break;
        case EjectingNote:
          transportIO.setTransportSpeed(-Constants.Transport.ejectNoteSpeed);
          break;
        case SweepTransport:
          transportIO.setTransportSpeed(1.5);
          break;
        case LaunchingNote:
          transportIO.setTransportSpeed(Constants.Transport.launchNoteTransportSpeed);
          break;
        default:
          transportIO.setTransportSpeed(0.);
          break;
      }
    }
  }
}
