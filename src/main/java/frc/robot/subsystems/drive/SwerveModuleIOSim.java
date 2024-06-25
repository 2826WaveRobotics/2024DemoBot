package frc.robot.subsystems.drive;

import java.util.OptionalDouble;
import java.util.Queue;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/**
 * Physics sim implementation of module IO.
 * This isn't a real implementation, although it eventually probably should be.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
  protected final FlywheelSim driveMotor;
  protected final FlywheelSim turnMotor;

  private double drivePosition = 0; // Radians
  private double turnPosition = 0; // Radians
  private double driveVelocity = 0; // Radians per second
  private double turnVelocity = 0; // Radians per second

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private boolean usingPID = true;
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.5, 2.35, 0.61);
  // Velocity
  private final PIDController driveController = Constants.Swerve.driveConfig.getPIDController(0);
  // Position
  private final PIDController turnController = Constants.Swerve.angleConfig.getPIDController(0);

  double driveVelocityReference = 0;
  double turnAngleReference = 0;
  
  public SwerveModuleIOSim() {
    driveMotor = new FlywheelSim(DCMotor.getNeoVortex(1), Constants.Swerve.driveGearRatio, 0.025);
    turnMotor = new FlywheelSim(DCMotor.getNEO(1), Constants.Swerve.angleGearRatio, 0.004096955);

    driveController.disableContinuousInput();
    turnController.enableContinuousInput(-Math.PI * Constants.Swerve.angleGearRatio, Math.PI * Constants.Swerve.angleGearRatio);

    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();

    drivePositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(
      () -> {
        driveVelocity = driveMotor.getAngularVelocityRadPerSec();
        Double lastTimestamp = timestampQueue.peek();
        if(lastTimestamp == null) lastTimestamp = 0.0;
        drivePosition += driveVelocity * ((Logger.getRealTimestamp() / 1e6) - lastTimestamp) / 1000.0;
        return OptionalDouble.of(drivePosition);
      }
    );
    turnPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(
      () -> {
        turnVelocity = turnMotor.getAngularVelocityRadPerSec();
        Double lastTimestamp = timestampQueue.peek();
        if(lastTimestamp == null) lastTimestamp = 0.0;
        turnPosition += turnVelocity * ((Logger.getRealTimestamp() / 1e6) - lastTimestamp) / 1000.0;
        return OptionalDouble.of(turnPosition);
      }
    );
  }
  
  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.drivePositionRad = drivePosition / Constants.Swerve.driveGearRatio;
    inputs.driveVelocityRadPerSec = driveVelocity / Constants.Swerve.driveGearRatio;
    inputs.driveCurrentAmps = driveMotor.getCurrentDrawAmps();

    inputs.turnPosition = Rotation2d.fromRadians(turnPosition / Constants.Swerve.angleGearRatio);

    inputs.turnReportedAbsolutePosition = inputs.turnPosition;
    inputs.turnAbsolutePosition = inputs.turnPosition;
    
    inputs.turnVelocityRadPerSec = turnVelocity / Constants.Swerve.angleGearRatio;
    inputs.turnCurrentAmps = turnMotor.getCurrentDrawAmps();

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
      .mapToDouble((Double value) -> value / Constants.Swerve.driveGearRatio)
      .toArray();
    inputs.odometryTurnPositions = turnPositionQueue.stream()
      .map((Double value) -> Rotation2d.fromRadians(value / Constants.Swerve.angleGearRatio))
      .toArray(Rotation2d[]::new);
    
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();

    // Update the simulation
    if(usingPID) {
      double driveAppliedVolts =
        feedforward.calculate(driveVelocityReference) +
        driveController.calculate(inputs.driveVelocityRadPerSec, driveVelocityReference);
        
      driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
      driveMotor.setInputVoltage(driveAppliedVolts);

      double turnAppliedVolts = MathUtil.clamp(turnController.calculate(turnPosition, turnAngleReference), -12.0, 12.0);
      turnMotor.setInputVoltage(turnAppliedVolts);
    }
    driveMotor.update(0.02);
    turnMotor.update(0.02);
  }

  @Override
  public void setDriveVelocity(double rpm) {
    usingPID = true;
    driveVelocityReference = rpm * Constants.Swerve.driveGearRatio;
  }

  @Override
  public void setTurnAngle(Rotation2d angle) {
    usingPID = true;
    turnAngleReference = angle.getRadians() * Constants.Swerve.angleGearRatio;
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    // Not supported in simulation
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    // Not supported in simulation
  }

  @Override
  public void setCharacterizationDriveVoltage(double voltage) {
    usingPID = false;
    driveMotor.setInput(voltage);
  }
}