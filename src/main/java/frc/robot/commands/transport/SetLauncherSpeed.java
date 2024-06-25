package frc.robot.commands.transport;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;

/**
 * A command that sets the launcher speed and finishes when it's at the target speed (plus or minus 100 RPM).  
 */
public class SetLauncherSpeed extends Command {
  private double speed;
  private boolean adjustForSpeaker;

  /**
   * Creates a new SetLauncherSpeed.
   * @param speed The launcher speed in RPM
   * @param adjustForSpeaker If we should adjust the roller speeds so we can keep the notes level. Only used for the speaker shots.
   */
  public SetLauncherSpeed(double speed, boolean adjustForSpeaker) {
    this.speed = speed;
    this.adjustForSpeaker = adjustForSpeaker;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Launcher.getInstance().setLauncherSpeed(speed, adjustForSpeaker);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Launcher.getInstance().getTopSpeedRPM() - speed) < 200;
  }
}
