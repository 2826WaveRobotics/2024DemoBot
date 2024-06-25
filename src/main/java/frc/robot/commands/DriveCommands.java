package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private DriveCommands() {}

  // private static SlewRateLimiter xVelocityRateLimiter = new SlewRateLimiter(3.0);
  // private static SlewRateLimiter yVelocityRateLimiter = new SlewRateLimiter(3.0);
  private static SlewRateLimiter xVelocityRateLimiter = new SlewRateLimiter(10000000.0);
  private static SlewRateLimiter yVelocityRateLimiter = new SlewRateLimiter(10000000.0);
  private static SlewRateLimiter omegaRateLimiter = new SlewRateLimiter(3.0);

  private static boolean pathfinding = false;
  public static void setPathfinding(boolean pathfinding) {
    DriveCommands.pathfinding = pathfinding;
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Swerve swerve,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      BooleanSupplier reduceSpeedSupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier fieldRelativeSupplier) {
    return new InstantCommand(
      () -> {
        // We convert the joystick inputs to linear and angular velocities so we can separately square the magnitude and angle.
        // This allows for finer-grained control of the robot's motion.

        double xValue = xVelocityRateLimiter.calculate(xSupplier.getAsDouble());
        double yValue = yVelocityRateLimiter.calculate(ySupplier.getAsDouble());
        double omegaValue = omegaRateLimiter.calculate(omegaSupplier.getAsDouble());

        double speedMultiplier = (reduceSpeedSupplier.getAsBoolean() ? (1 / 3.) : 1.0);

        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(xValue, yValue), Constants.Swerve.stickDeadband) * speedMultiplier;
        Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        double omega = MathUtil.applyDeadband(omegaValue, Constants.Swerve.stickDeadband) * speedMultiplier;

        // If we're pathfinding, don't control the drivetrain unless we manually override it.
        // This is mostly a failsafe -- ideally, the driver wouldn't stop pathfinding with joysticks
        // since the command will still be running.
        if (pathfinding && linearMagnitude == 0 && omega == 0) {
          return;
        }
        pathfinding = false;

        // Square values while preserving sign on omega
        linearMagnitude = linearMagnitude * linearMagnitude;
        omega = Math.copySign(omega * omega, omega);

        // Calcaulate new linear velocity
        Translation2d linearVelocity = new Pose2d(
          new Translation2d(),
          linearDirection.minus(Rotation2d.fromDegrees(90))
        ).transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

        if(fieldRelativeSupplier.getAsBoolean()) {
          // Drive relative to the field

          // Convert to field relative speeds and send command

          // We use "Always blue origin" coordinates. For more information about why we flip for red, see the following documentation:
          // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin
          boolean isFlipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
          swerve.driveVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
              linearVelocity.getX() * Constants.Swerve.maxSpeed,
              linearVelocity.getY() * Constants.Swerve.maxSpeed,
              omega * Constants.Swerve.maxAngularVelocity,
              isFlipped
                ? swerve.getRotation().plus(new Rotation2d(Math.PI))
                : swerve.getRotation())
          );
        } else {
          // Drive relative to the robot

          // Convert to robot relative speeds and send command
          swerve.driveVelocity(new ChassisSpeeds(
            linearVelocity.getX() * Constants.Swerve.maxSpeed,
            linearVelocity.getY() * Constants.Swerve.maxSpeed,
            omega * Constants.Swerve.maxAngularVelocity
          ));
        }
      },
      swerve);
  }
}