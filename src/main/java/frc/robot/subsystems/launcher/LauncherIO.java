package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface LauncherIO {
  @AutoLog
  public static class LauncherIOInputs {
    public Rotation2d absoluteLauncherAngle = new Rotation2d();
    public Rotation2d launcherRelativeConchAngle = new Rotation2d();
    public double launcherAngleVeocityRPM = 0;
    public double topRollerSpeedRPM = 0;
    public double bottomRollerSpeedRPM = 0;
  }

  /** Runs the launch rollers at the specified speed in RPM. */
  public default void runRollers(double topRollerSpeed, double bottomRollerSpeed) {}

  /** Sets the current limit for the top and bottom rollers, in amps. */
  public default void setRollerCurrentLimit(int currentLimit) {}

  /** Sets the angle of the conch motor in rotations. */
  public default void setAngleReference(double rotations) {}

  /** Resets the launcher position based on the absolute encoder. */
  public default void resetToAbsolute() {}
  
  /** Updates the set of loggable inputs. */
  public default void updateInputs(LauncherIOInputs inputs) {}
}