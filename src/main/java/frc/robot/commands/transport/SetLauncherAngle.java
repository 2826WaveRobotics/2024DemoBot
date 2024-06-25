package frc.robot.commands.transport;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;

/**
 * A command that sets the launcher angle and finishes when it's at the target angle.  
 */
public class SetLauncherAngle extends Command {
  private double angle;

  /**
   * Creates a new SetLauncherAngle.
   * @param angle The launcher angle in degrees
   */
  public SetLauncherAngle(double angle) {
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Launcher.getInstance().setLauncherAngle(Rotation2d.fromDegrees(angle));
  }

  Debouncer stopMovingDebouncer = new Debouncer(0.1, DebounceType.kRising);

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopMovingDebouncer.calculate(Launcher.getInstance().getAngleVelocityRPM() < 100);
  }
}
