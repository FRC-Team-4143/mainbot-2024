// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.lib.subsystem.Subsystem;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.PickupConstants;
import frc.robot.Constants.PickupSettings;
import frc.robot.Constants.ShooterConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class PickupSubsystem extends Subsystem {

    public enum PickupMode {
        IDLE, 
        PICKUP, 
        TRANSFER, 
        CLEAN
    }

    // Singleton pattern
    private static PickupSubsystem shooterPickupInstance = null;
    private static PickupSubsystem mailmainPickupInstance = null;

    public static PickupSubsystem getShooterInstance() {
        if (shooterPickupInstance == null) {
            shooterPickupInstance = new PickupSubsystem(PickupConstants.SHOOTER_PICKUP, "shooter");
        }
        return shooterPickupInstance;
    }

    public static PickupSubsystem getMailmanInstance() {
        if (mailmainPickupInstance == null) {
            mailmainPickupInstance = new PickupSubsystem(PickupConstants.MAILMAN_PICKUP, "mailman");
        }
        return mailmainPickupInstance;
    }

    /**
     * Class Members
     */
    private PickupPeriodicIo io_;
    private CANSparkBase roller_motor_;
    private PickupSettings settings_;
    private TimeOfFlight note_sensor_;
    private String name_;

    private PickupSubsystem(PickupSettings settings, String name) {
        name_ = name;
        settings_ = settings;
        io_ = new PickupPeriodicIo();
        roller_motor_ = new CANSparkFlex(settings.ROLLER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

        if (settings.PICKUP_NOTE_SENSOR_ID >= 0) {
            note_sensor_ = new TimeOfFlight(settings.PICKUP_NOTE_SENSOR_ID);
            note_sensor_.setRangingMode(TimeOfFlight.RangingMode.Medium, PickupConstants.SENSOR_SAMPLE_TIME);
        } else {
            note_sensor_ = null;
        }

        reset();
    }

    @Override
    public void reset() {
        roller_motor_.setSmartCurrentLimit(PickupConstants.ROLLER_AMP_LIMIT);
        roller_motor_.setInverted(settings_.ROLLER_MOTOR_INVERTED);
        roller_motor_.burnFlash();
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        if (note_sensor_ != null) {
            io_.note_sensor_range_ = note_sensor_.getRange();
        } else {
            io_.note_sensor_range_ = 1000;
        }
    }

    @Override
    public void updateLogic(double timestamp) {
        if (io_.has_note_pickup_ && io_.note_sensor_range_ > ShooterConstants.NO_NOTE_RANGE) {
            io_.has_note_pickup_ = false;
        } else if (io_.has_note_pickup_ == false && io_.note_sensor_range_ < ShooterConstants.HAS_NOTE_RANGE) {
            io_.has_note_pickup_ = true;
        }

        switch (io_.pickup_mode_) {
            case PICKUP:
                setRollersForward();
                break;
            case TRANSFER:
                setRollersForward();

                break;
            case CLEAN:
                setRollersBackward();
                break;
            default:
                stopRollers();
                break;
        }
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        roller_motor_.set(io_.roller_speed_);
    }

    @Override
    public void outputTelemetry(double timestamp) {
        SmartDashboard.putBoolean(name_ + "/has_note", hasNote());
        SmartDashboard.putNumber(name_ + "/range", io_.note_sensor_range_);
    }

    public void tellShooterReady() {
    }

    /**
     * Runs the rollers forward
     */
    public void setRollersForward() {
        io_.roller_speed_ = PickupConstants.ROLLER_FORWARD;
    }

    /**
     * Runs the rollers backward
     */
    public void setRollersBackward() {
        io_.roller_speed_ = PickupConstants.ROLLER_REVERSE;
    }

    /**
     * Stops the rollers
     */
    public void stopRollers() {
        io_.roller_speed_ = 0.0;
    }
/**
     * Sets the mode of the pickup subsystem enum, like PickupMode.PICKUP
     * @see PickupMode
     * @param mode The mode to change the PickupSubsystem to
     */
    public void setPickupMode(PickupMode mode) {
        io_.pickup_mode_ = mode;
    }

    /**
     * Returns true if the pickup has a note
     * @return If the pickup has a note
     */
    public boolean hasNote() {
        return io_.has_note_pickup_;
    }

    public class PickupPeriodicIo implements Logged {
        @Log.File
        public boolean has_note_pickup_ = false;
        @Log.File
        public boolean has_note_reciever_ = false;
        @Log.File
        public double roller_speed_ = 0.0;
        @Log.File
        public PickupMode pickup_mode_ = PickupMode.IDLE;
        @Log.File
        public double note_sensor_range_ = 0.0;
    }

    @Override
    public Logged getLoggingObject() {
      return io_;
    }
}
