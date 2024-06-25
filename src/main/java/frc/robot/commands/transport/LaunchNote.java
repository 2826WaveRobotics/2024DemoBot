package frc.robot.commands.transport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.NoteState;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.transport.Transport.TransportState;

/**
 * A command that runs the transport for 0.5 seconds to launch a note.
 */
public class LaunchNote extends Command {
  private double m_startTime;

  /** Creates a new LaunchNote. */
  public LaunchNote() {
    if(Superstructure.getInstance().getNoteState() != NoteState.ReadyToLaunch) cancel();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Transport.getInstance().attemptTransitionToState(TransportState.LaunchingNote);

    m_startTime = Timer.getFPGATimestamp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Transport.getInstance().attemptTransitionToState(TransportState.Stopped);
  }

  @Override
  public void execute() {
    Transport.getInstance().attemptTransitionToState(TransportState.LaunchingNote);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentTime = Timer.getFPGATimestamp();
    return ((currentTime - m_startTime) > 0.5);
  }
}
