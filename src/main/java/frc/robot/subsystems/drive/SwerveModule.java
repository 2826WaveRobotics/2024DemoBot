package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Units;
import frc.lib.util.ShuffleboardContent;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {
  /**
   * The number of times per second we read odometry data.
   */
  static final double ODOMETRY_FREQUENCY = 250.0;

  private final SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
  public int moduleIndex;

  private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Measure<Velocity<Distance>> speedSetpoint = null; // Setpoint for closed loop control, null for open loop
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
  
  public SwerveModule(int index, SwerveModuleIO io) {
    this.io = io;
    moduleIndex = index;

    setBrakeMode(true);

    ShuffleboardContent.initSwerveModuleShuffleboard(this);
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /** Resets the relative angle encoder to the CANCoder's absolute position (on a real module). */
  public void resetToAbsolute() {
    io.resetToAbsolute();
  }
  
  public void periodic() {
    Logger.processInputs("Drive/Module" + getModuleName(), inputs);

    // Run closed loop turn control
    if (angleSetpoint != null) {
      io.setTurnAngle(angleSetpoint);

      // Run closed loop drive control
      // Only allowed if closed loop turn control is running
      if (speedSetpoint != null) {
        // Scale velocity based on turn error
        //
        // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        var adjustedSpeedSetpoint = speedSetpoint.times(
          Math.cos(getAngle().getRadians() - angleSetpoint.getRadians())
        );

        // Run drive controller
        double velocityRPM = 
          adjustedSpeedSetpoint.in(Units.MetersPerSecond) / // Meters per second
          (Constants.Swerve.wheelDiameter * Math.PI) * // Rotations pesr second
          60.; // Rotations per minute

        io.setDriveVelocity(velocityRPM);
      }
    }

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * Constants.Swerve.wheelDiameter / 2.0;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null

    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    angleSetpoint = optimizedState.angle;
    speedSetpoint = Units.MetersPerSecond.of(optimizedState.speedMetersPerSecond);

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop turn control
    angleSetpoint = new Rotation2d();

    // Open loop drive control
    io.setCharacterizationDriveVoltage(volts);
    speedSetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveVelocity(0.0);
    io.setTurnAngle(new Rotation2d());

    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    speedSetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * Constants.Swerve.wheelDiameter / 2.0;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * Constants.Swerve.wheelDiameter / 2.0;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the drive velocity in radians/sec. */
  public double getDriveVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  public String getModuleName() {
    return Constants.Swerve.moduleNames[moduleIndex];
  }

  public double getReportedAbsoluteAngleDegrees() {
    return inputs.turnReportedAbsolutePosition.getDegrees();
  }

  public double getAbsoluteAngleDegrees() {
    return inputs.turnAbsolutePosition.getDegrees();
  }

  public double getAppliedDriveCurrent() {
    return inputs.driveCurrentAmps;
  }
  
  public double getAppliedAngleCurrent() {
    return inputs.turnCurrentAmps;
  }

  public double getTurnVelocityRadPerSec() {
    return inputs.turnVelocityRadPerSec;
  }

  public double getDriveMotorVelocityRPM() {
    return edu.wpi.first.math.util.Units.radiansPerSecondToRotationsPerMinute(inputs.driveVelocityRadPerSec) * Constants.Swerve.driveGearRatio;
  }
}
