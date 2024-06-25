package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface LimelightIO {
  @AutoLog
  public static class LimelightIOInputs {
    public Pose2d pose = new Pose2d();
    public int tagCount = 0;
    public double avgTagDist = 0;
    public double timestampSeconds = 0;
    
    public boolean intakeNotePresent = false;
    public Rotation2d intakeNoteX = new Rotation2d();
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LimelightIOInputs inputs) {}

  public default void flashIntakeLimelight() {}
}