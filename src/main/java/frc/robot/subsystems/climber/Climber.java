package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private static Climber instance = null;
  public static Climber getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          instance = new Climber(new ClimberIOReal());
          return instance;  
        default:
          instance = new Climber(new ClimberIO() { });
          return instance;
      }
    }
    return instance;
  }

  private ClimberIO climberIO;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
  }

  /** Gets the current draw of the left climber motor in amps. */
  public boolean getLeftMotorStallingClimb() {
    return inputs.leftClimberCurrentDrawAmps > 5.0;
  }
  
  public boolean getLeftMotorStallingBottom() {
    return Math.abs(inputs.leftClimberSpeedRPM) < 60.;
  }
  
  /** Gets the current draw of the right climber motor in amps. */
  public boolean getRightMotorStallingClimb() {
    return inputs.rightClimberCurrentDrawAmps > 5.0;
  }
  
  /** Gets the current draw of the right climber motor in amps. */
  public boolean getRightMotorStallingBottom() {
    return Math.abs(inputs.rightClimberSpeedRPM) < 60.;
  }

  /** Resets the encoder value for the left motor to 0. */
  public void resetLeftEncoder() {
    climberIO.resetLeftEncoder();
  }

  /** Resets the encoder value for the right motor to 0. */
  public void resetRightEncoder() {
    climberIO.resetRightEncoder();
  }

  /**
   * Sets the left motor speed in RPM. Positive values move the elevator downward.
   * @param speed The target speed, in RPM.
   */
  public void setLeftSpeed(double speed) {
    climberIO.setLeftSpeed(speed);
  }
  /**
   * Sets the right motor speed in RPM. Positive values move the elevator downward.
   * @param speed
   */
  public void setRightSpeed(double speed) {
    climberIO.setRightSpeed(speed);
  }

  /**
   * Gets the left climber arm position.
   * @return
   */
  public double getLeftPosition() {
    return inputs.leftPosition;
  }

  /**
   * Gets the right climber arm position.
   */
  public double getRightPosition() {
    return inputs.rightPosition;
  }

  /**
   * Sets the right motor position in rotations, where 0 is the resting position and positive numbers are upward.
   * @param position
   */
  public void setRightPosition(double position) {
    climberIO.setRightPosition(position);
  }
  /**
   * Sets the left motor position in rotations, where 0 is the resting position and positive numbers are upward.
   * @param position
   */
  public void setLeftPosition(double position) {
    climberIO.setLeftPosition(position);
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public void useClimbingCurrentLimit() {
    climberIO.useCurrentLimit(Constants.Climber.climbingSmartCurrentLimit);
  }
  public void useResetCurrentLimit() {
    climberIO.useCurrentLimit(Constants.Climber.resetSmartCurrentLimit);
  }
}
