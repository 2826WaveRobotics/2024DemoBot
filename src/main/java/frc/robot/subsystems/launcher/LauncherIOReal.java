package frc.robot.subsystems.launcher;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class LauncherIOReal implements LauncherIO {
  private CANSparkMax topRollerMotor;
  private CANSparkMax bottomRollerMotor;
  private CANSparkMax angleLauncherMotor;

  private RelativeEncoder angleLauncherEncoder;
  private DutyCycleEncoder absoluteAngleLauncherEncoder;

  private final SparkPIDController topLaunchRollerPIDController;
  private final SparkPIDController bottomLaunchRollerPIDController;
  private final SparkPIDController anglePIDController;

  private final RelativeEncoder topRollerEncoder;
  private final RelativeEncoder bottomRollerEncoder;

  public LauncherIOReal() {
    // Instantiate member variables and necessary code
    topRollerMotor = new CANSparkMax(Constants.Launcher.topRollerCANID, CANSparkMax.MotorType.kBrushless);
    bottomRollerMotor = new CANSparkMax(Constants.Launcher.bottomRollerCANID, CANSparkMax.MotorType.kBrushless);
    angleLauncherMotor = new CANSparkMax(Constants.Launcher.angleMotorCANID, CANSparkMax.MotorType.kBrushless);

    topLaunchRollerPIDController = topRollerMotor.getPIDController();
    bottomLaunchRollerPIDController = bottomRollerMotor.getPIDController();

    anglePIDController = angleLauncherMotor.getPIDController();

    Constants.Launcher.rollerConfig.configure(topRollerMotor, topLaunchRollerPIDController, "top launcher roller");
    Constants.Launcher.rollerConfig.configure(bottomRollerMotor, bottomLaunchRollerPIDController, "bottom launcher roller");
    
    Constants.Launcher.angleConfig.configure(angleLauncherMotor, anglePIDController, false, "launcher angle motor");
    angleLauncherMotor.setInverted(Constants.Launcher.invertAngle);

    angleLauncherEncoder = angleLauncherMotor.getEncoder();
    absoluteAngleLauncherEncoder = new DutyCycleEncoder(Constants.Launcher.absoluteEncoderDIOPort);

    topRollerEncoder = topRollerMotor.getEncoder();
    bottomRollerEncoder = bottomRollerMotor.getEncoder();

    resetToAbsolute();
    
    angleLauncherMotor.setSoftLimit(
      SoftLimitDirection.kForward,
      (float)(Constants.Launcher.softStopMarginLow.getRotations() * Constants.Launcher.angleMotorGearboxReduction)
    );
    angleLauncherMotor.setSoftLimit(
      SoftLimitDirection.kReverse,
      (float)((1 - Constants.Launcher.softStopMarginHigh.getRotations()) * Constants.Launcher.angleMotorGearboxReduction)
    );
    angleLauncherMotor.burnFlash();
  }
  
  @Override
  public void resetToAbsolute() {
    Rotation2d absolutePosition = getAbsoluteLauncherAngle().minus(Constants.Launcher.angleOffset);

    if(Constants.enableNonEssentialShuffleboard) {
      SmartDashboard.putNumber("Launcher absolutePosition", absolutePosition.getDegrees());
      SmartDashboard.putNumber("Launcher getAbsoluteLauncherAngle", getAbsoluteLauncherAngle().getDegrees());
    }
    double rotations = absolutePosition.getRotations() % 1.;
    if(
      rotations <
      -Constants.Launcher.softStopMarginHigh.getRotations() / 2
    ) rotations = rotations + 1;
    angleLauncherEncoder.setPosition(rotations * Constants.Launcher.angleMotorGearboxReduction);
  }

  private Rotation2d getAbsoluteLauncherAngle() {
    return Rotation2d.fromRotations(absoluteAngleLauncherEncoder.getAbsolutePosition());
  }

  private Rotation2d getLauncherConchAngle() {
    return Rotation2d.fromRotations((angleLauncherEncoder.getPosition() / Constants.Launcher.angleMotorGearboxReduction) % 1.);
  }

  @Override
  public void setRollerCurrentLimit(int currentLimit) {
    topRollerMotor.setSmartCurrentLimit((int)(currentLimit / 1.2), currentLimit, 0);
    bottomRollerMotor.setSmartCurrentLimit((int)(currentLimit / 1.2), currentLimit, 0);
  }

  private double oldRotations = 0.;

  @Override
  public void setAngleReference(double rotations) {
    if(rotations == oldRotations) return;
    oldRotations = rotations;

    if(Constants.enableNonEssentialShuffleboard) {
      SmartDashboard.putNumber("Launcher angle reference", rotations * 360.);
    }
    double setpoint = rotations * Constants.Launcher.angleMotorGearboxReduction;
    Logger.recordOutput("Launcher/RotationSetpoint", setpoint);
    anglePIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    inputs.absoluteLauncherAngle = getAbsoluteLauncherAngle();
    inputs.launcherRelativeConchAngle = getLauncherConchAngle();
    inputs.launcherAngleVeocityRPM = angleLauncherEncoder.getVelocity();
    inputs.topRollerSpeedRPM = topRollerEncoder.getVelocity();
    inputs.bottomRollerSpeedRPM = bottomRollerEncoder.getVelocity();
    
    if(Constants.enableNonEssentialShuffleboard) {
      SmartDashboard.putNumber("Launcher encoder angle reading", getLauncherConchAngle().getDegrees());
    }
  }

  private double oldTopRollerSpeed = 0.0;
  private double oldBottomRollerSpeed = 0.0;

  @Override
  public void runRollers(double topRollerSpeed, double bottomRollerSpeed) {
    if(topRollerSpeed != oldTopRollerSpeed) {
      topLaunchRollerPIDController.setReference(topRollerSpeed, CANSparkMax.ControlType.kVelocity);
      oldTopRollerSpeed = topRollerSpeed;
    }
    if(bottomRollerSpeed != oldBottomRollerSpeed) {
      bottomLaunchRollerPIDController.setReference(bottomRollerSpeed, CANSparkMax.ControlType.kVelocity);
      oldBottomRollerSpeed = bottomRollerSpeed;
    }
  }
}