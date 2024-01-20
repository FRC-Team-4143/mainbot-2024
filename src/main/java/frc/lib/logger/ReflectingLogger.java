package frc.lib.logger;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DataLogManager;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.ParameterizedType;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class ReflectingLogger<T> {
    protected Map<Field, T> fieldClassMap = new LinkedHashMap<>();
    protected Map<String, DataLogEntry> fieldLogEntryMap = new LinkedHashMap<>();

    /**
     * M
     * 
     * @param dataClasses
     */
    public ReflectingLogger(List<T> dataClasses) {
        this(dataClasses, DataLogManager.getLogDir());
    }

    /**
     * broadest constructor for the reflecting logger
     * 
     * @param dataClasses  the list of data classes to log on
     * @param loggingDir   the directory to log to
     * @param allowRethrow whether the logger is allowed to throw a runtime error if
     *                     the file cannot be opened
     */
    public ReflectingLogger(List<T> dataClasses, String loggingDir) {
        // begin the WPI logger
        DataLogManager.start(loggingDir);

        // generate map of subsystem IO's and fields
        for (T dataClass : dataClasses) {
            if (dataClass == null) {
                DataLogManager.log("null data class from subsystem");

            } else {
                for (Field field : dataClass.getClass().getFields()) {
                    fieldClassMap.put(field, dataClass);
                }
            }
        }

        // create the base file and header
        generateFields();
    }

    /**
     * Function for generating and writing the log fields to the WPILog file
     * 
     * @param loggingFile  the file to generate the header into and to use for
     *                     logging
     * @param allowRethrow whether the logger should allow throwing a runtime error
     *                     if the file cannot be opened
     */
    protected void generateFields() {

        // Set up custom log entries
        DataLog log = DataLogManager.getLog();

        // Write field names
        for (Map.Entry<Field, T> entry : fieldClassMap.entrySet()) {
            String full_name = entry.getKey().getName() + "." + entry.getKey().getType().getCanonicalName();
            try {
                // Null check the fields
                if (entry.getKey().get(entry.getValue()) == null) {
                    DataLogManager.log("Null data class member " + full_name);

                    // Handle Protobuf serializable types
                } else if (StructSerializable.class.isAssignableFrom(entry.getKey().getType())) {
                    // Create the log entry
                    var log_entry = StructLogEntry.create(log, full_name,
                            (Struct<T>) entry.getKey().getType().getDeclaredField("struct").get(entry.getValue()));

                    // put the datalog into the log map
                    fieldLogEntryMap.put(full_name, log_entry);

                    // Handle floating point types
                } else if (entry.getKey().get(entry.getValue()) instanceof Number) {
                    fieldLogEntryMap.put(full_name, new DoubleLogEntry(log, full_name));

                    // Handle boolean types
                } else if (entry.getKey().get(entry.getValue()) instanceof Boolean) {
                    fieldLogEntryMap.put(full_name, new BooleanLogEntry(log, full_name));

                    // Handle other types
                } else {
                    throw new UnsupportedOperationException();
                }
            } catch (IllegalArgumentException | IllegalAccessException | NoSuchFieldException | SecurityException e) {
                DataLogManager
                        .log("Failed to add data class member " + full_name + " to reflecting logger");
            }
        }

    }

    /**
     * Function that writes the next update to the log file. It reads a list of data
     * classes and dumps their contents into the opened file
     * 
     * @param dataClasses   the list of data classes to log
     * @param fpgaTimestamp the current FPGA time of the system to record
     */
    public void update(List<T> dataClasses, double fpgaTimestamp) {

        // generate map of subsystem IO's and fields
        for (T dataClass : dataClasses) {
            if (dataClass == null)
                continue;
            for (Field field : dataClass.getClass().getFields()) {
                fieldClassMap.put(field, dataClass);
            }
        }

        logMap(fpgaTimestamp);
    }

    protected void createLogEntry(Map.Entry<Field, T> entry, String full_name) throws IllegalAccessException, NoSuchFieldException {
        // Get the data
        Object log_data = entry.getKey().get(entry.getValue());

        // Handle null
        if (log_data == null) {
            DataLogManager.log("Attempted to log item with null value " + full_name);
            return; // For now do nothing

            // Handle serializable types
        } else if (StructSerializable.class.isAssignableFrom(entry.getKey().getType())) {
            ((StructLogEntry) fieldLogEntryMap.get(full_name)).append(
                    (Struct<T>) entry.getKey().getType().getDeclaredField("struct").get(entry.getValue()));

            // Handle floating point types
        } else if (log_data instanceof Number) {
            ((DoubleLogEntry) fieldLogEntryMap.get(full_name)).append((double) log_data);

            // Handle boolean types
        } else if (log_data instanceof Boolean) {
            ((BooleanLogEntry) fieldLogEntryMap.get(full_name)).append((boolean) log_data);

        } else {
            throw new UnsupportedOperationException();
        }

    }

    /**
     * Internal function for taking the class field map and writing it to the opened
     * file
     * 
     * @param fpgaTimestamp, the current timestamp to write in for the file update
     */
    protected void logMap(double fpgaTimestamp) {
        // for all fields in map generate
        for (Map.Entry<Field, T> entry : fieldClassMap.entrySet()) {
            // lookup the name
            String full_name = entry.getKey().getName() + "." + entry.getKey().getType().getCanonicalName();

            // Attempt to append subsystem IO value
            try {
                // check if this is a fixed size array type
                if (entry.getKey().getType().isArray()) {
                    final int array_len = Array.getLength(entry.getKey().get(entry.getValue()));
                    for (int i = 0; i < array_len; i++) {
                        String field_name = full_name + i;
                        createLogEntry(entry, full_name);
                    }
                } else {
                    // otherwise create the single entry
                    createLogEntry(entry, full_name);
                }

            } catch (IllegalArgumentException | IllegalAccessException | NoSuchFieldException | SecurityException e) {
                e.printStackTrace();
            }

        }

    }

    public synchronized void close() {
        DataLogManager.stop();
    }
}
