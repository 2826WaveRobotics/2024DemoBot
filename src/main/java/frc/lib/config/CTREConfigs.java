package frc.lib.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class CTREConfigs {
  public CANcoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveCanCoderConfig = new CANcoderConfiguration();

    /* Swerve CANCoder Configuration */
    MagnetSensorConfigs magnetSensorConfig = new MagnetSensorConfigs();
    magnetSensorConfig.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
    magnetSensorConfig.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
    swerveCanCoderConfig.withMagnetSensor(magnetSensorConfig);
  }
}
