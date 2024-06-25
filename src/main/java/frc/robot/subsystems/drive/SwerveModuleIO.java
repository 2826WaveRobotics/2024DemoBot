package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
  @AutoLog
  public static class SwerveModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveCurrentAmps = 0.0;

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnReportedAbsolutePosition = new Rotation2d();
    
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnCurrentAmps = 0.0;
    
    public double driveAppliedVolts = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  /** Run the drive motor at the specified velocity, given in RPM. */
  public default void setDriveVelocity(double rpm) {}

  /** Sets the turn motor to the specified angle. */
  public default void setTurnAngle(Rotation2d angle) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Directly sets the drive voltage for system identification purposes. */
  public default void setCharacterizationDriveVoltage(double voltage) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}

  /** Resets the relative angle encoder to the CANCoder's absolute position (on a real module). */
  public default void resetToAbsolute() {}
}