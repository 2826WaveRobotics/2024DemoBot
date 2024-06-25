package frc.robot.subsystems.drive;

import java.util.OptionalDouble;
import java.util.Queue;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GyroIOSim implements GyroIO {
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private double yaw = 0; // Degrees
  private double yawVelocity = 0; // Degrees per second

  public GyroIOSim() {
    yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(
      () -> {
        return OptionalDouble.of(yaw + yawVelocity * ((Logger.getRealTimestamp() / 1e6) - yawTimestampQueue.peek()) / 1000.0);
      }
    );
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = Rotation2d.fromDegrees(yaw);
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity);

    inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions = yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}