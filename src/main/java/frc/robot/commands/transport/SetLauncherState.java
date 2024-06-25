package frc.robot.commands.transport;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;

/**
 * A command that sets the launcher state and finishes when both the angle and speed are at the target values.
 */
public class SetLauncherState extends ParallelRaceGroup {
  /**
   * Creates a new SetLauncherState.
   * @param state The launcher state
   */
  public SetLauncherState(LauncherState state) {
    addRequirements(Launcher.getInstance());
    addCommands(
      new WaitCommand(1.5),
      new ParallelCommandGroup(
        new SetLauncherAngle(state.angleDegrees),
        new SetLauncherSpeed(state.speed, state.adjustForSpeaker)
      )
    );
  }
}
