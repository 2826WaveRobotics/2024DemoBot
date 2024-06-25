package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.config.CTREConfigs;
import frc.lib.config.SwerveModuleConstants;
import frc.robot.Constants;

import java.util.OptionalDouble;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO,
 * NEO 550, or NEO Vortex), and CANCoder absolute encoder.
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class SwerveModuleIOSparkMax implements SwerveModuleIO {
  private static final CTREConfigs ctreConfigs = new CTREConfigs();

  protected final CANSparkMax driveSparkMax;
  protected final CANSparkMax turnSparkMax;

  private final SparkPIDController drivePIDController;
  private final SparkPIDController turnPIDController;
  
  private final CANcoder cancoder;
  private final StatusSignal<Double> turnAbsolutePosition;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final Rotation2d absoluteEncoderOffset;

  public SwerveModuleIOSparkMax(SwerveModuleConstants moduleConstants) {
    driveSparkMax = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    turnSparkMax = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    cancoder = new CANcoder(moduleConstants.cancoderID);
    cancoder.getConfigurator().apply(ctreConfigs.swerveCanCoderConfig);
    cancoder.optimizeBusUtilization();
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnAbsolutePosition.setUpdateFrequency(50.0);

    absoluteEncoderOffset = moduleConstants.angleOffset;

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    drivePIDController = driveSparkMax.getPIDController();
    turnPIDController = turnSparkMax.getPIDController();

    Constants.Swerve.driveConfig.configure(driveSparkMax, drivePIDController, false, "swerve drive motor " + moduleConstants.driveMotorID);
    Constants.Swerve.angleConfig.configure(turnSparkMax, turnPIDController, false, "swerve angle motor " + moduleConstants.angleMotorID);

    turnPIDController.setPositionPIDWrappingEnabled(true);
    turnPIDController.setPositionPIDWrappingMinInput(0.0);
    turnPIDController.setPositionPIDWrappingMaxInput(Constants.Swerve.angleGearRatio);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    resetToAbsolute();
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int)(1000.0 / SwerveModule.ODOMETRY_FREQUENCY));
    turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int)(1000.0 / SwerveModule.ODOMETRY_FREQUENCY));

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();

    // Odometry update logic initialization
    
    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();

    drivePositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(
      () -> {
        double value = driveEncoder.getPosition();
        if (driveSparkMax.getLastError() == REVLibError.kOk) {
          return OptionalDouble.of(value);
        } else {
          return OptionalDouble.empty();
        }
      }
    );
    turnPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(
      () -> {
        double value = turnRelativeEncoder.getPosition();
        if (driveSparkMax.getLastError() == REVLibError.kOk) {
          return OptionalDouble.of(value);
        } else {
          return OptionalDouble.empty();
        }
      }
    );
  }

  /** Resets the relative angle encoder to the CANCoder's absolute position. */
  @Override
  public void resetToAbsolute() {
    turnRelativeEncoder.setPosition(
      (turnAbsolutePosition.getValueAsDouble() - absoluteEncoderOffset.getRotations()) * Constants.Swerve.angleGearRatio
    );
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    turnAbsolutePosition.refresh();
    
    inputs.drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition()) / Constants.Swerve.driveGearRatio;
    inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / Constants.Swerve.driveGearRatio;
    inputs.driveCurrentAmps = driveSparkMax.getOutputCurrent();

    inputs.turnReportedAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnAbsolutePosition = inputs.turnReportedAbsolutePosition.minus(absoluteEncoderOffset);
    
    inputs.turnPosition = Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / Constants.Swerve.angleGearRatio);
    inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity()) / Constants.Swerve.angleGearRatio;
    inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();

    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
      .mapToDouble((Double value) -> Units.rotationsToRadians(value) / Constants.Swerve.driveGearRatio)
      .toArray();
    inputs.odometryTurnPositions = turnPositionQueue.stream()
      .map((Double value) -> Rotation2d.fromRotations(value / Constants.Swerve.angleGearRatio))
      .toArray(Rotation2d[]::new);
    
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  double oldReferenceRPM = 0;
  @Override
  public void setDriveVelocity(double rpm) {
    if(rpm == oldReferenceRPM) return;
    oldReferenceRPM = rpm;

    drivePIDController.setReference(rpm * Constants.Swerve.driveGearRatio, ControlType.kVelocity);
  }

  double oldTurnAngle = 0;
  @Override
  public void setTurnAngle(Rotation2d angle) {
    if(angle.getRadians() == oldTurnAngle) return;
    oldTurnAngle = angle.getRadians();

    turnPIDController.setReference(angle.getRotations() * Constants.Swerve.angleGearRatio, ControlType.kPosition);
  }

  boolean oldDriveBrakeModeEnabled = false;
  @Override
  public void setDriveBrakeMode(boolean enable) {
    if(enable == oldDriveBrakeModeEnabled) return;
    oldDriveBrakeModeEnabled = enable;

    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  boolean oldTurnBrakeModeEnabled = false;
  @Override
  public void setTurnBrakeMode(boolean enable) {
    if(enable == oldTurnBrakeModeEnabled) return;
    oldTurnBrakeModeEnabled = enable;
    
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setCharacterizationDriveVoltage(double voltage) {
    driveSparkMax.setVoltage(voltage);
  }
}