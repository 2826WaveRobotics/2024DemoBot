package frc.robot.subsystems.transport;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants;

public class TransportIOReal implements TransportIO {
  private CANSparkMax topTransportMotor;
  private CANSparkMax bottomTransportMotor;

  private SparkPIDController bottomTransportPIDController;
  private SparkPIDController topTransportPIDController;

  public TransportIOReal() {
    topTransportMotor = new CANSparkMax(Constants.Transport.topTransportMotorCANID, CANSparkMax.MotorType.kBrushless);
    bottomTransportMotor = new CANSparkMax(Constants.Transport.bottomTransportMotorCANID, CANSparkMax.MotorType.kBrushless);

    topTransportPIDController = topTransportMotor.getPIDController();
    bottomTransportPIDController = bottomTransportMotor.getPIDController();

    Constants.Transport.transportMotorConfig.configure(topTransportMotor, topTransportPIDController, "top transport motor");
    Constants.Transport.transportMotorConfig.configure(bottomTransportMotor, bottomTransportPIDController, "bottom transport motor");
  }

  /**
   * Gets the rpm that an intake motor should run at, based on the target speed of the edge of the wheels and belt.  
   * This is effectively the speed that the notes move at.
   * @param speedMetersPerSecond The speed the the belt/edge of intake wheels will move at, in meters per second.
   * This is effectively the speed that the note moves.
   * @return
   */
  private double getTransportMotorRPM(double speedMetersPerSecond, double gearRatio) {
    double wheelCircumference = Constants.Transport.rollerDiameterMeters * Math.PI;
    double revolutionsPerSecond = speedMetersPerSecond / wheelCircumference * gearRatio;
    return revolutionsPerSecond * 60;
  }

  /**
   * The slew rate limiter for the transport speed.
   */
  private SlewRateLimiter transportSlewRateLimiter = new SlewRateLimiter(100000);

  private double oldLimitedSpeed = 0.0;

  /**
   * Sets the transport speeds. Speeds are how fast the edge wheels will move at, in meters per second.
   * This is effectively the speed that the note moves.  
   * @param speedMetersPerSecond The speed that the intake and transport wheels will spin, in meters per second.
   */
  @Override
  public void setTransportSpeed(double speedMetersPerSecond) {
    double limitedSpeed = transportSlewRateLimiter.calculate(speedMetersPerSecond);
    if(speedMetersPerSecond == 0.0) limitedSpeed = 0.0;

    Logger.recordOutput("Transport/TopMotorCurrentDraw", topTransportMotor.getOutputCurrent());
    Logger.recordOutput("Transport/BottomMotorCurrentDraw", bottomTransportMotor.getOutputCurrent());

    if(limitedSpeed == oldLimitedSpeed) return;
    oldLimitedSpeed = limitedSpeed;
    double speedRPM = getTransportMotorRPM(limitedSpeed, Constants.Transport.transportGearRatio);
    topTransportPIDController.setReference(-speedRPM, ControlType.kVelocity);
    bottomTransportPIDController.setReference(-speedRPM, ControlType.kVelocity);
  }
}
