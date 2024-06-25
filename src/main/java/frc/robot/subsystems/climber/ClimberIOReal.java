package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants;

public class ClimberIOReal implements ClimberIO {
  private CANSparkMax leftClimberMotor;
  private CANSparkMax rightClimberMotor;

  private RelativeEncoder leftClimberEncoder;
  private RelativeEncoder rightClimberEncoder;

  private SparkPIDController leftPIDController;
  private SparkPIDController rightPIDController;

  public ClimberIOReal() {
    leftClimberMotor = new CANSparkMax(Constants.Climber.leftClimberMotorCANID, CANSparkMax.MotorType.kBrushless);
    rightClimberMotor = new CANSparkMax(Constants.Climber.rightClimberMotorCANID, CANSparkMax.MotorType.kBrushless);

    leftClimberEncoder = leftClimberMotor.getEncoder();
    rightClimberEncoder = rightClimberMotor.getEncoder();

    leftPIDController = leftClimberMotor.getPIDController();
    rightPIDController = rightClimberMotor.getPIDController();

    Constants.Climber.motorConfig.configure(rightClimberMotor, rightPIDController, "right climber motor");    
    Constants.Climber.motorConfig.configure(leftClimberMotor, leftPIDController, "left climber motor");
  }
  
  /** Resets the encoder value for the left motor to 0. */
  @Override
  public void resetLeftEncoder() {
    leftClimberEncoder.setPosition(0);
  }

  /** Resets the encoder value for the right motor to 0. */
  @Override
  public void resetRightEncoder() {
    rightClimberEncoder.setPosition(0);
  }

  private boolean currentlyControllingSpeedLeft = false;
  private boolean currentlyControllingSpeedRight = false;

  private double oldLeftSpeed = 0.0;
  private double oldRightSpeed = 0.0;
  
  /**
   * Sets the left motor speed in RPM. Positive values move the climber downward.
   * @param speed The target speed, in RPM.
   */
  @Override
  public void setLeftSpeed(double speed) {
    if(speed == oldLeftSpeed) return;
    oldLeftSpeed = speed;

    if(!currentlyControllingSpeedLeft) leftClimberMotor.stopMotor();
    currentlyControllingSpeedLeft = true;

    leftPIDController.setReference(-speed, ControlType.kVelocity, 1);
  }
  /**
   * Sets the right motor speed in RPM. Positive values move the climber downward.
   * @param speed
   */
  @Override
  public void setRightSpeed(double speed) {
    if(speed == oldRightSpeed) return;
    oldRightSpeed = speed;

    if(!currentlyControllingSpeedRight) rightClimberMotor.stopMotor();
    currentlyControllingSpeedRight = true;

    rightPIDController.setReference(speed, ControlType.kVelocity, 1);
  }

  private double oldLeftPosition = 0.0;
  private double oldRightPosition = 0.0;

  /**
   * Sets the right motor position in rotations, where 0 is the resting position and positive numbers are upward.
   * @param position
   */
  @Override
  public void setRightPosition(double position) {
    if(position == oldRightPosition) return;
    oldRightPosition = position;

    if(currentlyControllingSpeedRight) rightClimberMotor.stopMotor();
    currentlyControllingSpeedRight = false;
    rightPIDController.setReference(position, ControlType.kPosition, 0);
  }
  
  /**
   * Sets the left motor position in rotations, where 0 is the resting position and positive numbers are upward.
   * @param position
   */
  @Override
  public void setLeftPosition(double position) {
    if(position == oldLeftPosition) return;
    oldLeftPosition = position;

    if(currentlyControllingSpeedLeft) leftClimberMotor.stopMotor();
    currentlyControllingSpeedLeft = false;
    leftPIDController.setReference(position, ControlType.kPosition, 0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftClimberCurrentDrawAmps = leftClimberMotor.getOutputCurrent();
    inputs.rightClimberCurrentDrawAmps = rightClimberMotor.getOutputCurrent();

    inputs.leftPosition = leftClimberEncoder.getPosition();
    inputs.rightPosition = rightClimberEncoder.getPosition();

    inputs.leftClimberSpeedRPM = leftClimberEncoder.getVelocity();
    inputs.rightClimberSpeedRPM = rightClimberEncoder.getVelocity();
  }

  int oldSmartLimit = 0;

  @Override
  public void useCurrentLimit(int smartLimit) {
    if(smartLimit != oldSmartLimit) {
      leftClimberMotor.setSmartCurrentLimit((int)(smartLimit / 1.5), smartLimit, 0);
      oldSmartLimit = smartLimit;
    }
  }
}
