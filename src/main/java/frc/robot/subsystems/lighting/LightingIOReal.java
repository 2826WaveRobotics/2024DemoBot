package frc.robot.subsystems.lighting;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.subsystems.lighting.Lighting.LightState;

public class LightingIOReal implements LightingIO {
    /**
     * The Serial device for the lighting arduino.
     */
    private static SerialPort serialLightingArduino = null;

    private void openSerial() {
        if(failedCount < 5) {
            try {
                serialLightingArduino = new SerialPort(9600, Port.kUSB2);
            } catch(Exception e) {
                try {
                    serialLightingArduino = new SerialPort(9600, Port.kUSB1);
                } catch(Exception e2) {
                    failedCount++;
                    DriverStation.reportError("Failed to create lighting Arduino - " + e.getLocalizedMessage() + ", " + e2.getLocalizedMessage(), false);
                }
            }
        }
    }

    public LightingIOReal() {
        openSerial();
    }

    /**
     * Gets a lighting update with the current lighting state.  
     * Message format is 0xFF, 1 byte for the light state, 1 byte for the alliance, and 1 byte for the robot speed
     * (as a ratio of the full speed).
     * Checks and casts are done on both sides.
     * 
     * @param robotSpeed The robot speed in meters per second.
     */
    private byte[] getUpdate(double robotSpeed, LightState lightState) {
        byte[] data = new byte[4];
        data[0] = (byte)0xFF;

        data[1] = (byte)lightState.ordinal();

        // This value matches the Arduino's enum definitions for the alliance.
        data[2] = (byte)(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 1);
        // 5.4 meters per second is our maximum speed.
        // This is just a rough conversion to 0-254, which is what we send to the Arduino.
        data[3] = (byte)(Math.min(robotSpeed / 5.4, 1.0) * 254);
        return data;
    }

    private int failedCount = 0;
    private byte[] lastData = new byte[4];

    @Override
    public void setLightState(double robotSpeed, LightState lightState) {
        if(serialLightingArduino == null) openSerial();
        if(serialLightingArduino == null) return;

        var data = getUpdate(robotSpeed, lightState);
        if(data == lastData) return;
        
        Logger.recordOutput("Lighting/SentMessage", data);
        int bytesWritten = serialLightingArduino.write(data, 4);
        if(bytesWritten != data.length && failedCount < 10) {
            failedCount++;
            DriverStation.reportError("Failed to write to lighting Arduino", false);
        }
    }
}
