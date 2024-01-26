package frc.lib.subsystem;

import java.util.ArrayList;

import frc.lib.logger.Logable.LogData;
import frc.lib.logger.ReflectingLogger;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;

public abstract class SubsystemManager {

    // Supposedly 1 is a good starting point, but can increase if we have issues
    private static final int START_THREAD_PRIORITY = 99;

    protected ReflectingLogger<LogData> reflectingLogger;
    protected ArrayList<Subsystem> subsystems;
    protected Notifier loopThread;

    public SubsystemManager() {
        // Initialize the subsystem list
        subsystems = new ArrayList<>();

        // create the thread to loop the subsystems and mark as daemon thread so
        // the robot program can properly stop
        loopThread = new Notifier(this::doControlLoop);
        // loopThread.setDaemon(true);
    }

    private void doControlLoop() {

        // For each subsystem get incoming data
        double timestamp = Timer.getFPGATimestamp();
        for (Subsystem subsystem : subsystems) {
            try {
                subsystem.readPeriodicInputs(timestamp);
            } catch (Exception e) {
                e.printStackTrace();
                DataLogManager.log(subsystem.getClass().getCanonicalName() + "failed to read inputs");
            }

        }

        // Now update the logic for each subsystem to allow I/O to relax
        timestamp = Timer.getFPGATimestamp();
        for (Subsystem subsystem : subsystems) {
            try {
                subsystem.updateLogic(timestamp);
            } catch (Exception e) {
                e.printStackTrace();
                DataLogManager.log(subsystem.getClass().getCanonicalName() + "failed to update logic");
            }

        }

        // Finally write the outputs to the actuators
        timestamp = Timer.getFPGATimestamp();
        for (Subsystem subsystem : subsystems) {
            try {
                subsystem.writePeriodicOutputs(timestamp);
            } catch (Exception e) {
                e.printStackTrace();
                DataLogManager.log(subsystem.getClass().getCanonicalName() + "failed to write outputs");
            }
        }

        // Run the logger!
        // runLog(Timer.getFPGATimestamp());

    }

    /**
     * Completes the subsystem registration process and begins calling each
     * subsystem in a loop
     */
    protected void completeRegistration() {
        loopThread.startPeriodic(.1);
    }

    /**
     * Helper function to collect log data classes for recording
     * 
     * @param timestamp the timestamp logging was started at from the FPGA
     */
    protected void runLog(double timestamp) {
        // Check if the logger is valid first
        if (reflectingLogger != null) {
            // If it is valid, collect the subsystem I/Os
            ArrayList<LogData> logs = new ArrayList<>();
            for (Subsystem subsystem : subsystems) {
                logs.add(subsystem.getLogger());
            }

            // Log the data
            reflectingLogger.update(logs, timestamp);
        }
    }

    /**
     * Once the subsystems have been registered, call this function to init logging
     * capabilities
     */
    public void initLogfile() {
        ArrayList<LogData> logs = new ArrayList<>();
        for (Subsystem subsystem : subsystems) {
            logs.add(subsystem.getLogger());
        }
        // reflectingLogger = new ReflectingLogger<>(logs);
    }

    /**
     * When ready to stop a log file from being written to, call this function to
     * close the log and shut the log process down. This call is reversible via the
     * initLogFile method.
     */
    public void stopLog() {
        reflectingLogger = null;
    }

    /**
     * When ready to put telemetry out to smartdashboard, call this function to
     * trigger each subsystem to publish its held data. This is supposed to be
     * called by robotPeriodic so telemetry is output in any mode.
     */
    public void outputTelemetry() {
        for (Subsystem subsystem : subsystems) {
            subsystem.outputTelemetry(Timer.getFPGATimestamp());
        }
    }

    /**
     * If subsystems all need to be reset before a robot mode change, call this
     * function to cleanly handle resetting them together. If only one subsystem
     * needs to be reset, that can be accessed through the getInstance method.
     */
    public void reset() {
        for (Subsystem subsystem : subsystems) {
            subsystem.reset();
        }
    }

}
