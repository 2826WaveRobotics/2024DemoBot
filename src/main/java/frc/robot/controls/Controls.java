package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.StoreModuleOffsets;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.climber.ClimberControls;
import frc.robot.commands.transport.SetLauncherState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.transport.Transport.TransportState;

/**
 * This class manages the controls of the robot. We use it so all the button bindings can be in one place.
 */
public class Controls {
    private static Controls instance = null;
    public static Controls getInstance() {
        if (instance == null) {
            instance = new Controls();
        }
        return instance;
    }
    
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private Controls() {
        // This is a singleton class.
    }

    /**
     * Configures the controls for the robot. This is where we bind commands to buttons and add joystick actions.
     */
    public void configureControls() {
        Swerve swerveSubsystem = Swerve.getInstance();
        Launcher launcherSubsystem = Launcher.getInstance();
        Transport transportSubsystem = Transport.getInstance();
        
        DriverStation.silenceJoystickConnectionWarning(true);

        /*//////////////////////////*/
        /*     Driver Controls      */
        /*//////////////////////////*/
        BooleanSupplier fieldRelative = driver.leftBumper();
        swerveSubsystem.setDefaultCommand(
            DriveCommands.joystickDrive(
                swerveSubsystem,
                () ->  driver.getLeftX(),
                () -> -driver.getLeftY(),
                driver.leftTrigger(0.2),
                () -> -driver.getRightX(),
                () -> !fieldRelative.getAsBoolean()
            )
        );

        driver.x().onTrue(new InstantCommand(swerveSubsystem::stopWithX, swerveSubsystem));
        driver.start().onTrue(new InstantCommand(swerveSubsystem::resetRotation, swerveSubsystem).ignoringDisable(true));
        driver.back().onTrue(new InstantCommand(swerveSubsystem::resetToAbsolute));
        
        driver.a().onTrue(new InstantCommand(() -> {
            if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
                swerveSubsystem.setPose(new Pose2d(
                    new Translation2d(1.37, 5.55),
                    new Rotation2d()
                ));
            } else {
                swerveSubsystem.setPose(new Pose2d(
                    new Translation2d(Constants.fieldLengthMeters - 1.37, 5.55),
                    Rotation2d.fromDegrees(180)
                ));
            }
        }));

        /*//////////////////////////*/
        /*    Operator Controls     */
        /*//////////////////////////*/
        
        transportSubsystem.setDefaultCommand(new TeleopIntake(() -> -operator.getLeftY()));
    
        operator.b().onTrue(new InstantCommand(() -> transportSubsystem.attemptTransitionToState(TransportState.IntakingNote)));
        operator.a().onTrue(new InstantCommand(() -> transportSubsystem.attemptTransitionToState(TransportState.Stopped)));

        BooleanSupplier testMode = operator.x();
        BooleanSupplier notInTestMode = () -> !testMode.getAsBoolean();

        // Test angle mode
        // launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(25));
        operator.povUp()
            .whileTrue(new RepeatCommand(new InstantCommand(() -> launcherSubsystem.setLauncherAngle(launcherSubsystem.launcherAngle.plus(Rotation2d.fromDegrees(0.15))))).onlyIf(testMode))
            .onTrue(new SetLauncherState(Constants.Controls.DPadUpPreset).onlyIf(notInTestMode));
        operator.povDown()
            .whileTrue(new RepeatCommand(new InstantCommand(() -> launcherSubsystem.setLauncherAngle(launcherSubsystem.launcherAngle.minus(Rotation2d.fromDegrees(0.15))))).onlyIf(testMode))
            .onTrue(new SetLauncherState(Constants.Controls.DPadDownPreset).onlyIf(notInTestMode));
        operator.povRight()
            .whileTrue(new RepeatCommand(new InstantCommand(() -> launcherSubsystem.setLauncherSpeed(launcherSubsystem.topRollerSpeed + 20, true))).onlyIf(testMode))
            .onTrue(new SetLauncherState(Constants.Controls.DPadRightPreset).onlyIf(notInTestMode));
        operator.povLeft()
            .whileTrue(new RepeatCommand(new InstantCommand(() -> launcherSubsystem.setLauncherSpeed(launcherSubsystem.topRollerSpeed - 20, true))).onlyIf(testMode))
            .onTrue(new SetLauncherState(Constants.Controls.DPadLeftPreset).onlyIf(notInTestMode));

        operator.rightTrigger(0.2).whileTrue(new RepeatCommand(new InstantCommand(
            () -> Transport.getInstance().attemptTransitionToState(TransportState.LaunchingNote)
        ))).onFalse(new InstantCommand(
            () -> Transport.getInstance().attemptTransitionToState(TransportState.Stopped)
        ));
        
        // Lob shot
        operator.leftTrigger(0.2).onTrue(new SetLauncherState(Constants.Controls.LobShotState));
        
        operator.start().whileTrue(Commands.startEnd(
            () -> launcherSubsystem.setLauncherSpeed(-6800, false),
            () -> launcherSubsystem.setLauncherSpeed(0, false)
        ));

        Climber.getInstance().setDefaultCommand(ClimberControls.teleopControls(
            operator.leftBumper(),
            operator.rightBumper(),
            operator.x()
        ));

        operator.back().whileTrue(new StoreModuleOffsets().onlyIf(DriverStation::isTest));
    }
}
