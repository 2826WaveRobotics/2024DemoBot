package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.transport.Transport.TransportState;

public class TeleopIntake extends Command {
    private DoubleSupplier intakeOverride;

    public TeleopIntake(DoubleSupplier intakeOverride) {
        addRequirements(Transport.getInstance());

        this.intakeOverride = intakeOverride;
    }

    private double lastSetSpeed = 0;

    @Override
    public void execute() {
        if(DriverStation.isAutonomous()) return;

        Transport transport = Transport.getInstance();
        
        double intakeSpeed = MathUtil.applyDeadband(intakeOverride.getAsDouble(), 0.25);

        if(lastSetSpeed != intakeSpeed) {
            lastSetSpeed = intakeSpeed;

            if(intakeSpeed == 0) {
                transport.attemptTransitionToState(TransportState.Stopped);
            } else {
                transport.attemptTransitionToState(TransportState.OperatorOverride);
                transport.setOperatorOverrideSpeedMetersPerSecond(intakeSpeed * Constants.Transport.intakeSpeed);
            }
        }
    }
}
