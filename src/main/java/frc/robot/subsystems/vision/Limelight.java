package frc.robot.subsystems.vision;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.lib.LimelightHelpers;
import frc.lib.util.ShuffleboardContent;

public class Limelight extends SubsystemBase {
  private static Limelight instance = null;
  public static Limelight getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          instance = new Limelight(new LimelightIOReal());
          return instance;
        default:
          // Replayed or sim robot, disable IO implementations
          instance = new Limelight(new LimelightIO() { });
          return instance;
      }
    }
    return instance;
  }

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.  
   * We use an extremely low standard deviation for the heading state because we trust our gyro more than our vision for heading measurements.
   */
  public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(1));

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   * We use an extremely high standard deviation for the heading measurement because we trust our gyro more than our vision for heading measurements.
   */
  public static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(90));
  
  private LimelightIO limelightIO;
  private LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();

  private Limelight(LimelightIO limelightIO) {
    this.limelightIO = limelightIO;

    if(!DriverStation.isFMSAttached()) {
      var limelightFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer);
      CameraServer.startAutomaticCapture(limelightFeed);
    }
  }

  private void updateOdometryPoseFromVisionMeasurements() {
    Logger.recordOutput("Odometry/LimelightPoseEstimate", inputs.pose);

    boolean discardMeasurement = false;

    ChassisSpeeds robotRelativeSpeeds = Swerve.getInstance().getRobotRelativeSpeeds();
    if (
      Math.abs(robotRelativeSpeeds.omegaRadiansPerSecond) > Units.degreesToRadians(720) ||
      Math.abs(robotRelativeSpeeds.vxMetersPerSecond) > 5.0 ||
      Math.abs(robotRelativeSpeeds.vyMetersPerSecond) > 5.0
    ) discardMeasurement = true;
    
    if(inputs.tagCount <= 0) discardMeasurement = true;

    // Scale the vision measurement expected standard deviation (0.1m by default) exponentially by the distance 
    double standardDeviationScalar = Math.max(0.02, 0.5 * Math.pow(1.2, inputs.avgTagDist)) / inputs.tagCount;

    Logger.recordOutput("Odometry/LimelightPoseEstimateUsed", !discardMeasurement);

    if(!discardMeasurement) {
      Swerve.getInstance().addVisionMeasurement(inputs.pose, inputs.timestampSeconds, standardDeviationScalar);
    }
  }

  @Override
  public void periodic() {
    limelightIO.updateInputs(inputs);
    Logger.processInputs("Limelight", inputs);

    updateOdometryPoseFromVisionMeasurements();
  }

  public void initiaize() {
    ShuffleboardContent.competitionTab.addCamera("Intake feed", "Intake Limelight", "http://limelight-intake.local:5800/stream.mjpg")
      .withPosition(0, 0)
      .withSize(5, 4)
      .withProperties(Map.of("Show Crosshair", false, "Show Controls", false));
    
    // Forward the Limelight camera ports
    // More information: https://docs.limelightvision.io/docs/docs-limelight/getting-started/best-practices#event-preparation-checklist
    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight.local", port);
      PortForwarder.add(port + 10, "limelight-intake.local", port);
    }
  }
}
