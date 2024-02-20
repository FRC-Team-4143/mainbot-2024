package frc.lib.logger;

import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * A base level class for inheritance to allow for the creation
 * of a list of loggable classes.
 */
public interface Logable {
    /**
     * A base level data class for use capturing logging data
     * this inheritance can be used to combine lower level logging in subsystems
     * as well as inside major components
     */
    public static class LogData {

    }

    public abstract LoggableInputs getLogger();
}