package frc.robot.subsystems.drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drive.FieldRelativeAcceleration;
import frc.lib.drive.FieldRelativeVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Limelight;

public class Swerve extends SubsystemBase {
  private static Swerve instance = null;
  public static Swerve getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          instance = new Swerve(
            new GyroIOPigeon2(),
            new SwerveModuleIO[] {
              new SwerveModuleIOSparkMax(Constants.Swerve.mod0Constants),
              new SwerveModuleIOSparkMax(Constants.Swerve.mod1Constants),
              new SwerveModuleIOSparkMax(Constants.Swerve.mod2Constants),
              new SwerveModuleIOSparkMax(Constants.Swerve.mod3Constants)
            }
          );
          return instance;
        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          instance = new Swerve(
            new GyroIO() {},
            new SwerveModuleIO[] { new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim() }
          );
          return instance;
        default:
          // Replayed robot, disable IO implementations
          instance = new Swerve(
            new GyroIO() {},
            new SwerveModuleIO[] { new SwerveModuleIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {} }
          );
          return instance;
      }
    }
    return instance;
  }

  static final Lock odometryLock = new ReentrantLock();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator swerveOdometry;
  private SwerveModule[] swerveModules;

  
  private Rotation2d rawGyroRotation = new Rotation2d();
  /**
   * The last known module positions.  
   * Used for delta tracking.
   */
  private SwerveModulePosition[] lastModulePositions;

  private Swerve(
    GyroIO gyroIO,
    SwerveModuleIO[] moduleIOs
  ) {
    this.gyroIO = gyroIO;
    
    swerveModules = new SwerveModule[moduleIOs.length];
    for (int i = 0; i < moduleIOs.length; i++) {
      swerveModules[i] = new SwerveModule(i, moduleIOs[i]);
    }
    
    SparkMaxOdometryThread.getInstance().start();

    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModulePositions[i] = swerveModules[i].getPosition();
    }
    
    lastModulePositions = swerveModulePositions;

    Pose2d startPose = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d());

    kinematics = new SwerveDriveKinematics(Constants.Swerve.modulePositions);

    swerveOdometry = new SwerveDrivePoseEstimator(
      kinematics,
      getYaw(),
      swerveModulePositions,
      startPose,
      Limelight.stateStdDevs,
      Limelight.visionMeasurementStdDevs
    );
    
    resetRotation();

    if(Constants.enableNonEssentialShuffleboard) {
      Shuffleboard.getTab("Notes").addString("Odometry position", () -> ("(" + getPose().getX() + ", " + getPose().getY() + ")"));
      
      Shuffleboard.getTab("Notes").addNumber("Speaker distance", () -> {
        Swerve swerve = Swerve.getInstance();

        Translation2d currentPosition = swerve.getPose().getTranslation();
        
        double speakerInward = -0.1;
        double speakerY = 5.55;

        boolean isBlueAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
        Translation2d targetLocation = isBlueAlliance ? new Translation2d(speakerInward, speakerY) : new Translation2d(Constants.fieldLengthMeters - speakerInward, speakerY);

        Translation2d relativeTargetLocation = targetLocation.minus(currentPosition);
        double distance = relativeTargetLocation.getNorm();

        return distance;
      });
      Shuffleboard.getTab("Notes").addNumber("Lob distance", () -> {
        Swerve swerve = Swerve.getInstance();

        Translation2d currentPosition = swerve.getPose().getTranslation();

        double targetX = 1.38;
        double targetY = 7.06;

        boolean isBlueAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
        Translation2d targetLocation = isBlueAlliance ? new Translation2d(targetX, targetY) : new Translation2d(Constants.fieldLengthMeters - targetX, targetY);

        Translation2d relativeTargetLocation = targetLocation.minus(currentPosition);
        double distance = relativeTargetLocation.getNorm();

        return distance;
      });
    }
  }

  /** Resets the relative angle encoder to the CANCoder's absolute position on every module. */
  public void resetToAbsolute() {
    for(var mod : swerveModules) {
      mod.resetToAbsolute();
    }
  }

  private double robotSpeed = 0.0;
  private FieldRelativeVelocity velocity = new FieldRelativeVelocity();
  private FieldRelativeVelocity lastVelocity = new FieldRelativeVelocity();
  private FieldRelativeAcceleration acceleration = new FieldRelativeAcceleration();

  /**
   * Gets the absolute speed of the robot in meters per second.
   * @return
   */
  @AutoLogOutput(key = "Drive/Speed")
  public double getRobotSpeed() {
    return robotSpeed;
  }
  public FieldRelativeVelocity getFieldRelativeVelocity() {
    return velocity;
  }
  public FieldRelativeAcceleration getFieldRelativeAcceleration() {
    return acceleration;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
  }

  public void addVisionMeasurement(Pose2d pose, double timestampSeconds, double standardDeviationScalar) {
    swerveOdometry.addVisionMeasurement(
      pose,
      timestampSeconds,
      Limelight.visionMeasurementStdDevs.times(standardDeviationScalar)
    );
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      states[i] = swerveModules[i].getPosition();
    }
    return states;
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void driveVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.Swerve.maxSpeed);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < swerveModules.length; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = swerveModules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    driveVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < swerveModules.length; i++) {
      headings[i] = Constants.Swerve.modulePositions[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      states[i] = swerveModules[i].getState();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return swerveOdometry.getEstimatedPosition();
  }

  /**
   * Returns the current odometry rotation.
   * NOTE: This isn't necessarily the same as the gyro angle.
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Resets the odometry to face the current direction.
   */
  public void resetRotation() {
    setPose(new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public double[] getOffsets() {
    return new double[] {
      swerveModules[0].getReportedAbsoluteAngleDegrees(),
      swerveModules[1].getReportedAbsoluteAngleDegrees(),
      swerveModules[2].getReportedAbsoluteAngleDegrees(),
      swerveModules[3].getReportedAbsoluteAngleDegrees()
    };
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    swerveOdometry.addVisionMeasurement(visionPose, timestamp);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveModules) {
      states[mod.moduleIndex] = mod.getState();
    }
    return states;
  }

  /**
   * Gets the current yaw of the gyro, relative to when we last zeroed it.  
   * Positive Z/yaw is left.
   * @return
   */
  public Rotation2d getYaw() {
    return gyroInputs.yawPosition;
  }

  Tracer timeTracer = new Tracer();

  @Override
  public void periodic() {
    double startTime = Logger.getRealTimestamp();
    timeTracer.clearEpochs();

    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    timeTracer.addEpoch("Gyro updates");
    for (SwerveModule module : swerveModules) {
      module.updateInputs();
    }
    timeTracer.addEpoch("Module input updates");
    odometryLock.unlock();

    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (SwerveModule module : swerveModules) {
      module.periodic();
    }
    timeTracer.addEpoch("Module periodic");

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (SwerveModule module : swerveModules) {
        module.stop();
      }
      timeTracer.addEpoch("Module stop");
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry.
    // We use many samples per update to vastly increase the accuracy of the odometry.

    double[] sampleTimestamps = swerveModules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < swerveModules.length; moduleIndex++) {
        modulePositions[moduleIndex] = swerveModules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
          modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
          modulePositions[moduleIndex].angle
        );
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      swerveOdometry.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
    
    timeTracer.addEpoch("Odometry updates");

    double executionTime = Logger.getRealTimestamp() - startTime;
    if(executionTime > 10000) { // More than 0.01 seconds
      System.out.println("Swerve time took over 0.01 seconds: " + ((Logger.getRealTimestamp() - startTime) / 1000000) + "s");
      timeTracer.printEpochs();
    }
    Logger.recordOutput("Drive/ExecutionTimeMS", executionTime * 0.001);

    // Update speed, velocity, and accceleration
    
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), rawGyroRotation);
    robotSpeed = Math.sqrt(
      chassisSpeeds.vxMetersPerSecond * chassisSpeeds.vxMetersPerSecond +
      chassisSpeeds.vyMetersPerSecond * chassisSpeeds.vyMetersPerSecond
    );

    velocity = new FieldRelativeVelocity(chassisSpeeds, gyroInputs.yawPosition);
    acceleration = new FieldRelativeAcceleration(velocity, lastVelocity, 0.02);
    lastVelocity = velocity;
  }
}
