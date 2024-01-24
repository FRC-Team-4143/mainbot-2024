package frc.lib.subsystem;

import frc.lib.logger.Logable;

public abstract class Subsystem implements Logable {

    /**
     * Reads all sensors and stores periodic data
     */
    public abstract void readPeriodicInputs(double timestamp);

    /**
     * Computes updated outputs for the actuators
     */
    public abstract void updateLogic(double timestamp);

    /**
     * Writes the periodic outputs to actuators (motors and etc...)
     */
    public abstract void writePeriodicOutputs(double timestamp);

    /**
     * Outputs all logging information to the SmartDashboard
     */
    public abstract void outputTelemetry(double timestamp);

    /**
     * Called to reset and configure the subsystem
     */
    public abstract void reset();

    /**
     * Called to get access to all of the periodic class data
     */
    public abstract LogData getLogger();
}
