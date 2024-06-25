package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// List class features here, including any motors, sensors, and functionality:
// 2 Launcher motors to score notes
// 1 motor to aim (~30-60 degree window)
// 1 CAN Coder for getting the absolute angle
public class Launcher extends SubsystemBase {
  private static Launcher instance = null;
  public static Launcher getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          instance = new Launcher(new LauncherIOReal());   
          return instance;
        default:
          instance = new Launcher(new LauncherIO() {});   
          return instance;
      }
    }
    return instance;
  }

  private LauncherIO launcherIO;
  private LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();
  
  private Launcher(LauncherIO launcherIO) {
    this.launcherIO = launcherIO;
    
    setLauncherSpeed(Constants.Launcher.idleRollerVelocity, false);
  }
  
  public static class LauncherState {
    public double speed;
    public double angleDegrees;
    public boolean adjustForSpeaker;

    public LauncherState(double speed, double angleDegrees) {
      this(speed, angleDegrees, true);
    }

    public LauncherState(double speed, double angleDegrees, boolean adjustForSpeaker) {
      this.speed = speed;
      this.angleDegrees = angleDegrees;
      this.adjustForSpeaker = adjustForSpeaker;
    }
  }

  /**
   * The current target launcher angle.
   */
  @AutoLogOutput(key = "Launcher/TargetAngle")
  public Rotation2d launcherAngle = Rotation2d.fromDegrees(45);
  @AutoLogOutput(key = "Launcher/TopRollerSpeed")
  public double topRollerSpeed = 1200;
  @AutoLogOutput(key = "Launcher/BottomRollerSpeed")
  public double bottomRollerSpeed = 1200;

  /**
   * Sets the launcher state.
   * @param state
   */
  public void setLauncherState(LauncherState state) {
    setLauncherSpeed(state.speed, state.adjustForSpeaker);
    setLauncherAngle(Rotation2d.fromDegrees(state.angleDegrees));
  }

  /**
   * Calculates the required conch angle from the wanted launcher angle.
   * @param angle
   */
  public void setLauncherAngle(Rotation2d angle) {
    launcherAngle = angle;
    if(Constants.enableNonEssentialShuffleboard) {
      SmartDashboard.putNumber("LauncherAngle", launcherAngle.getDegrees());
    }
    
    // All length units here are in inches
    double conchToPivotDistance = 5.153673;
    double pivotToConchReactionBarDistance = 4.507;
    double angleOffsetRadians = Units.degreesToRadians(23.53);
    // Law of cosines
    double requiredRadius = Math.sqrt(
      conchToPivotDistance * conchToPivotDistance + pivotToConchReactionBarDistance * pivotToConchReactionBarDistance
      - 2 * conchToPivotDistance * pivotToConchReactionBarDistance * Math.cos(Math.max(angle.getRadians() - angleOffsetRadians, 0.0))
    );
    
    double lowRadius = 0.1875; // in, from CAD
    double radiusIncrease = 3.4375; // in, from CAD: (4 -0.375/2-0.5/2)-(1/8)

    double conchAngleRadians = (requiredRadius - lowRadius) / radiusIncrease * (Math.PI * 2);
    if(conchAngleRadians < Constants.Launcher.softStopMarginLow.getRadians()) {
      // The angle is lower than we can achieve
      conchAngleRadians = Constants.Launcher.softStopMarginLow.getRadians();
    }
    if(conchAngleRadians > Math.PI * 2 - Constants.Launcher.softStopMarginHigh.getRadians()) {
      // The angle is higher than we can achieve
      conchAngleRadians = Math.PI * 2 - Constants.Launcher.softStopMarginHigh.getRadians();
    }

    launcherIO.setAngleReference(Rotation2d.fromRadians(conchAngleRadians).getRotations());
  }

  /**
   * Resets the launcher to the encoder's absolute position.
   */
  public void resetToAbsolute() {
    launcherIO.resetToAbsolute();
  }

  /**
   * Sets the launch roller speed.
   * @param speed
   * @param adjustToLevelNote If we should adjust the roller speeds so we can keep the notes level. Only used for the speaker shots.
   */ 
  public void setLauncherSpeed(double speed, boolean adjustToLevelNote) {
    if(adjustToLevelNote) {
      bottomRollerSpeed = speed * 0.65;
    } else {
      bottomRollerSpeed = speed;
    }
    topRollerSpeed = speed;
  }

  public double getAngleVelocityRPM() {
    return inputs.launcherAngleVeocityRPM;
  }

  public double getTopSpeedRPM() {
    return inputs.topRollerSpeedRPM;
  }
  public double getBottomSpeedRPM() {
    return inputs.bottomRollerSpeedRPM;
  }

  public boolean atSetpoints() {
    // The mechanism only allows a maximum of 4650 RPM.
    return
      getAngleVelocityRPM() < 60.0 &&
      Math.abs(getTopSpeedRPM() - Math.min(topRollerSpeed, 4650)) < 100.0 &&
      Math.abs(getBottomSpeedRPM() - Math.min(bottomRollerSpeed, 4650)) < 100.0;
  }

  public void useAutoCurrentLimits() {
    launcherIO.setRollerCurrentLimit(Constants.Launcher.rollerCurrentLimitForAuto);
  }
  public void useTeleopCurrentLimits() {
    launcherIO.setRollerCurrentLimit(Constants.Launcher.rollerCurrentLimitForTeleop);
  }

  @Override
  public void periodic() {
    launcherIO.updateInputs(inputs);
    Logger.processInputs("Launcher/Inputs", inputs);

    launcherIO.runRollers(topRollerSpeed, bottomRollerSpeed);

    if(Constants.enableNonEssentialShuffleboard) {
      SmartDashboard.putNumber("Absolute launcher angle", inputs.absoluteLauncherAngle.getDegrees());
    }
    
    if(Constants.enableNonEssentialShuffleboard) {
      SmartDashboard.putNumber("TopRollerTargetSpeed", topRollerSpeed);
      SmartDashboard.putNumber("BottomRollerTargetSpeed", bottomRollerSpeed);
    }
  }
}
