package frc.robot.subsystems.lighting;

import frc.robot.subsystems.lighting.Lighting.LightState;

public interface LightingIO {
    /** Sets the lighting state based on the given robot speed and note state. */
    public default void setLightState(double robotSpeed, LightState lightState) {}
}