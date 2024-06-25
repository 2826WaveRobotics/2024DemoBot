package frc.lib.util;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.CANSparkMaxUtil.Usage;

public class CANSparkMaxConfig {
    private class PIDValues {
        public int slot;
        public double p;
        public double i;
        public double d;
        public double f;
        public double iZone;

        public PIDValues(int slot, double p, double i, double d, double f, double iZone) {
            this.slot = slot;
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
            this.iZone = iZone;
        }
    }

    /**
     * The idle mode of the motor.
     */
    public CANSparkMax.IdleMode idleMode;
    /**
     * The smart current limit in Amps.
     * <p>The motor controller will reduce the controller voltage output to avoid surpassing this limit. This limit is
     * enabled by default and used for brushless only. This limit is highly recommended when using the NEO brushless motor.  
     * <p>The NEO Brushless Motor has a low internal resistance, which can mean large current spikes that could
     * be enough to cause damage to the motor and controller. This current limit provides a smarter strategy
     * to deal with high current draws and keep the motor and controller operating in a safe region.
     */
    public int smartCurrentLimit;

    /**
     * The voltage that this motor will run at so we can compensate.
     */
    public double voltageCompensation;

    /**
     * The CAN bus usage of this motor.
     */
    public Usage usage;

    public ArrayList<PIDValues> pidConfigurations = new ArrayList<PIDValues>();

    public CANSparkMaxConfig(
        CANSparkMax.IdleMode idleMode,
        int smartCurrentLimit,
        double voltageCompensation,
        Usage usage
    ) {
        this.idleMode = idleMode;
        this.smartCurrentLimit = smartCurrentLimit;
        this.voltageCompensation = voltageCompensation;
        this.usage = usage;
    }

    /**
     * Adds a PID slot to this Spark Max's configuration.
     * @param slot
     * @param p
     * @param i
     * @param d
     * @param f
     */
    public CANSparkMaxConfig configurePIDSlot(int slot, double p, double i, double d, double f) {
        configurePIDSlot(slot, p, i, d, f, 0);
        return this;
    }

    /**
     * Adds a PID slot to this Spark Max's configuration.
     * @param slot
     * @param p
     * @param i
     * @param d
     * @param f
     * @param iZone
     */
    public CANSparkMaxConfig configurePIDSlot(int slot, double p, double i, double d, double f, double iZone) {
        pidConfigurations.add(new PIDValues(slot, p, i, d, f, iZone));
        return this;
    }

    /**
     * Gets the PID controller for the given slot.  
     * This is useful for configuring simulation models.
     * @param slot
     * @return
     */
    public PIDController getPIDController(int slot) {
        PIDValues values = pidConfigurations.get(slot);
        PIDController controller = new PIDController(values.p, values.i, values.d, 0.02);
        controller.setIntegratorRange(-values.iZone, values.iZone);
        return controller;
    }

    /**
     * Configures the given Spark Max and Spark PID controller from this configuration and burns the settings to flash.
     * @param spark
     * @param pidController
     * @param name A human-readable device name.
     */
    public void configure(CANSparkMax spark, SparkPIDController pidController, String name) {
        configure(spark, pidController, true, name);
    }

    /**
     * Configures the given Spark Max and Spark PID controller from this configuration.
     * @param spark
     * @param pidController
     * @param burnFlash If we should burn the new settings to flash.
     * @param name A human-readable device name.
     */
    public void configure(CANSparkMax spark, SparkPIDController pidController, boolean burnFlash, String name) {
        logErrorIfNotOK(spark.restoreFactoryDefaults(), "restoreFactoryDefaults", name);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(spark, usage);
        logErrorIfNotOK(
            spark.setSmartCurrentLimit((int)Math.floor(smartCurrentLimit / 2.), smartCurrentLimit, 0),
            "setSmartCurrentLimit", name
        );

        logErrorIfNotOK(spark.setIdleMode(idleMode), "setIdleMode", name);

        for(PIDValues values : pidConfigurations) {
            logErrorIfNotOK(pidController.setP(values.p, values.slot), "setP", name);
            logErrorIfNotOK(pidController.setI(values.i, values.slot), "setI", name);
            logErrorIfNotOK(pidController.setD(values.d, values.slot), "setD", name);
            logErrorIfNotOK(pidController.setFF(values.f, values.slot), "setFF", name);
            logErrorIfNotOK(pidController.setIZone(values.iZone, values.slot), "setIZone", name);
        }

        logErrorIfNotOK(spark.enableVoltageCompensation(voltageCompensation), "enableVoltageCompensation", name);
        
        // if(burnFlash) logErrorIfNotOK(spark.burnFlash(), "burnFlash", name);
    }

    private void logErrorIfNotOK(REVLibError error, String type, String name) {
        if(error != REVLibError.kOk) {
            DriverStation.reportError("Could not configure SPARK Max: " + type + " configuration failed on device " + name + "!", false);
        }
    }
}
