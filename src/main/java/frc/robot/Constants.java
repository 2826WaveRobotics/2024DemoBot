package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.util.CANSparkMaxConfig;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.subsystems.launcher.Launcher.LauncherState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  /**
   * The current mode of the robot.  
   * This _must_ be changed before running simulations or on the real robot.  
   * There may be a better way to do this, but we need to differentiate between replays and normal simulations which doesn't seem possible.
   */
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
  /** A situation that the robot is currently running in: real, simulation, or replay. */
  public enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  /**
   * The deadband used for controller triggers.
   */
  public static final double triggerDeadband = 0.03;

  public static final boolean enableNonEssentialShuffleboard = true;

  public static final double fieldLengthMeters = Units.feetToMeters(54. + 1./12);

  public static final class Controls {
    public static final LauncherState AmpState = new LauncherState(1780, 58.35, true);
    public static final LauncherState SpeakerCloseState = new LauncherState(4500, 59.95, true);
    public static final LauncherState LobShotState = new LauncherState(5000, 45, true);

    public static final LauncherState DPadLeftPreset = AmpState;
    public static final LauncherState DPadUpPreset = new LauncherState(2880, 42.1, true); // Podium speaker preset
    public static final LauncherState DPadDownPreset = new LauncherState(Constants.Launcher.idleRollerVelocity, 21, false); // Slow preset
    public static final LauncherState DPadRightPreset = SpeakerCloseState;
  }

  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final int pigeonID = 10;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(20);
    // distance between centers of right and left wheels on robot
    public static final double wheelBase = Units.inchesToMeters(21);

    /** The swerve drive wheel diameters, in meters. */
    public static final double wheelDiameter = Units.inchesToMeters(4.0); //4

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final String[] moduleNames = {"frontLeft", "frontRight", "backRight", "backLeft"};

    // The locations of the modules on the robot, in meters.
    public static final Translation2d[] modulePositions = {
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),   // fl
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),  // fr
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0), // br
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0)   // bl
    };

    /* Swerve Profiling Values */
    /**
     * The maximum robot movement speed in meters per second.
     */
    public static final double maxSpeed = 5.2;
    /**
     * The maximum robot angular velocity in radians per second.
     */
    public static final double maxAngularVelocity = 11.5;

    /**
     * The spark max config for the drive motors.
     */
    public static final CANSparkMaxConfig driveConfig = new CANSparkMaxConfig(
      CANSparkMax.IdleMode.kBrake,
      35,
      10.0,
      Usage.kAll
    ).configurePIDSlot(0, 0.0002, 0.0, 0.0, 1. / 6784 /* Free speed of a NEO Vortex */);
    
    /**
     * The spark max config for the angle motors.
     */
    public static final CANSparkMaxConfig angleConfig = new CANSparkMaxConfig(
      CANSparkMax.IdleMode.kBrake,
      30,
      
      10.0,
      Usage.kAll
    ).configurePIDSlot(0, 3.0, 2e-4, 0.02, 0.0);

    public static final Rotation2d[] swerveOffsets = getSwerveOffsets();
    public static final String swerveOffsetFileName = "swerveOffsets.txt";
    private static final Rotation2d[] getSwerveOffsets() {
      Rotation2d[] offsets = new Rotation2d[4];
      try (
        BufferedReader br = new BufferedReader(new FileReader(new File(Filesystem.getDeployDirectory(), swerveOffsetFileName)))
      ) {
        String line;
        int i = 0;
        while ((line = br.readLine()) != null) {
          if(line.length() == 0) continue; // Skip empty lines -- the last line might be empty
          offsets[i++] = Rotation2d.fromDegrees(Double.parseDouble(line));
        }
      } catch(IOException ioException) {
        DriverStation.reportError("ERROR: IO exception while reading swerve offsets: " + ioException.getLocalizedMessage(), false);
      }

      return offsets;
    }

    public static final SwerveModuleConstants mod0Constants =
      new SwerveModuleConstants(11, 12, 13, swerveOffsets[0]); /* Front left */
    public static final SwerveModuleConstants mod1Constants =
      new SwerveModuleConstants(21, 22, 23, swerveOffsets[1]); /* Front right */
    public static final SwerveModuleConstants mod2Constants =
      new SwerveModuleConstants(31, 32, 33, swerveOffsets[2]); /* Back right */
    public static final SwerveModuleConstants mod3Constants =
      new SwerveModuleConstants(41, 42, 43, swerveOffsets[3]); /* Back left */
  }

  public static final class Auto {
    public static final double kMaxSpeedMetersPerSecond = 3; //3.25
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class Launcher {
    public static final int absoluteEncoderDIOPort = 1;

    public static final int topRollerCANID = 59;
    public static final int bottomRollerCANID = 52;
    public static final int angleMotorCANID = 27;

    public static final double wheelRadiusMeters = Units.inchesToMeters(2);

    public static int rollerCurrentLimitForAuto = 30;
    public static int rollerCurrentLimitForTeleop = 25;
    public static CANSparkMaxConfig rollerConfig = new CANSparkMaxConfig(
      CANSparkMax.IdleMode.kCoast,
      rollerCurrentLimitForAuto,
      10.0,
      CANSparkMaxUtil.Usage.kAll
    ).configurePIDSlot(0, 0.0005, 1e-6, 0.0, 1 / 5700., 500);

    public static CANSparkMaxConfig angleConfig = new CANSparkMaxConfig(
      CANSparkMax.IdleMode.kBrake,
      15,
      10.0,
      CANSparkMaxUtil.Usage.kAll
    ).configurePIDSlot(0, 0.075, 0.0, -1e-7, 0.0);

    public static final double angleMotorGearboxReduction = 5.23 * 5.23 * 2.89;

    /**
     * The idle launch roller velocity in revolutions per minute.
     */
    public static final double idleRollerVelocity = 3000.;

    public static final Rotation2d softStopMarginLow = Rotation2d.fromDegrees(15);
    public static final Rotation2d softStopMarginHigh = Rotation2d.fromDegrees(75);

    /**
     * The conch angle offset. 0 degrees should be where the axle sits at the lowest point.
     */
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(42);

    /**
     * If we should invert the angle motor direction.
     */
    public static final boolean invertAngle = false;
  }

  public static final class NoteSensors {
    /**
     * The DIO port of the through beam sensor detecting if notes are in the intake.
     */
    public static final int intakeSensorDIOPort = 2;
    /**
     * The DIO port of the through beam sensor detecting if the note is in position.
     */
    public static final int noteInPositionSensorDIOPort = 6;
    /**
     * The DIO port of the through beam sensor detecting if the note is transitioning to the resting position.
     */
    public static final int noteInTransitionSensorDIOPort = 4;
  }

  public static final class Transport {
    public static final int topTransportMotorCANID = 45;
    public static final int bottomTransportMotorCANID = 54;

    public static final double rollerDiameterMeters = Units.inchesToMeters(1.525);

    public static final double transportGearRatio = 5; // : 1

    public static final CANSparkMaxConfig transportMotorConfig = new CANSparkMaxConfig(
      IdleMode.kCoast,
      20,
      10.0,
      CANSparkMaxUtil.Usage.kPositionOnly
    ).configurePIDSlot(0, 1e-4, 0.0, 0.0, 1. / 5700.);

    /**
     * The speed that notes are moved into the launcher to shoot, in meters per second.  
     * 3.8 m/s is roughly our max speed.
     */
    public static final double launchNoteTransportSpeed = 2.9;
    /**
     * The speed to move notes when ejecting.  
     * Notes are ejected when we have more than 1 note in the transport at a time.
     */
    public static final double ejectNoteSpeed = 0.5;
    /**
     * The intake speed (meaning the speed at the edge of the wheel/the speed that the belt moves at), in meters per second.  
     * 3.8 m/s is roughly our max speed.
     */
    public static final double intakeSpeed = 2.9;
  }

  public static final class Climber {
    public static final int leftClimberMotorCANID = 55;
    public static final int rightClimberMotorCANID = 56;

    public static final int climbingSmartCurrentLimit = 30;
    public static final int resetSmartCurrentLimit = 2;

    /**
     * The speed to run the climber motors at when climbing.
     */
    public static final double climberMotorSpeed = 4000;

    public static final CANSparkMaxConfig motorConfig = new CANSparkMaxConfig(
      IdleMode.kBrake,
      resetSmartCurrentLimit,
      10.0,
      Usage.kAll
    ).configurePIDSlot(0, 0.1, 0.0, 0.0, 0.0)         // Position controller
     .configurePIDSlot(1, 2e-6, 0.0, 0.0, 1. / 5600.); // Velocity controller
  }
}
