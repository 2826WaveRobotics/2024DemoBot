package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.LimelightHelpers;
import frc.robot.subsystems.drive.Swerve;

public class LimelightIOReal implements LimelightIO {
    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        LimelightHelpers.SetRobotOrientation(
            "limelight",
            Swerve.getInstance().getPose().getRotation().getDegrees(),
            Units.radiansToDegrees(Swerve.getInstance().getRobotRelativeSpeeds().omegaRadiansPerSecond),
            0, 0, 0, 0
        );
        LimelightHelpers.PoseEstimate limelightMeasurement =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        inputs.pose = limelightMeasurement.pose;
        inputs.avgTagDist = limelightMeasurement.avgTagDist;
        inputs.tagCount = limelightMeasurement.tagCount;
        inputs.timestampSeconds = limelightMeasurement.timestampSeconds;

        inputs.intakeNotePresent = LimelightHelpers.getTV("limelight-intake");
        inputs.intakeNoteX = Rotation2d.fromDegrees(LimelightHelpers.getTX("limelight-intake"));
    }
}
