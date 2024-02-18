// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.lib.subsystem.Subsystem;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import frc.robot.Constants.PickupConstants;
import frc.robot.Constants.PickupSettings;

public class PickupSubsystem extends Subsystem {

  private enum PickupMode {
    IDLE, PICKUP, TRANSFER, CLEAN
  }

  // Singleton pattern
  private static PickupSubsystem shooterPickupInstance = null;
  private static PickupSubsystem mailmainPickupInstance = null;

  public static PickupSubsystem getShooterInstance() {
    if (shooterPickupInstance == null) {
      shooterPickupInstance = new PickupSubsystem(PickupConstants.SHOOTER_PICKUP);
    }
    return shooterPickupInstance;
  }

  public static PickupSubsystem getMailmanInstance() {
    if (mailmainPickupInstance == null) {
      mailmainPickupInstance = new PickupSubsystem(PickupConstants.MAILMAN_PICKUP);
    }
    return mailmainPickupInstance;
  }

  /**
   * 
   */
  private PeriodicIo io_;
  private final CANSparkFlex roller_motor_;
  private final PickupSettings settings_;
  private final TimeOfFlight note_sensor_;

  /**
   * Constructor for the example subsystem. The constructor should create all
   * instances of the required hardware as well as the PeriodicIO class defined
   * below. This should not attempt to configure any of the hardware as that
   * should be done in the reset() function.
   */
  private PickupSubsystem(PickupSettings settings) {
    settings_ = settings;
    io_ = new PeriodicIo();
    roller_motor_ = new CANSparkFlex(settings.ROLLER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    reset();

    if (settings.PICKUP_NOTE_SENSOR_ID < 0) {
      note_sensor_ = new TimeOfFlight(settings.PICKUP_NOTE_SENSOR_ID);
      note_sensor_.setRangingMode(TimeOfFlight.RangingMode.Medium, PickupConstants.SENSOR_SAMPLE_TIME);
    } else {
      note_sensor_ = null;
    }
  }

  @Override
  /**
   * Inside this function should be logic and code to fully reset your subsystem.
   * This is called during initialization, and should handle I/O configuration and
   * initializing data members. pickup_rollers
   */
  public void reset() {
    io_ = new PeriodicIo();
    roller_motor_.setSmartCurrentLimit(PickupConstants.ROLLER_AMP_LIMIT);
    roller_motor_.setInverted(settings_.ROLLER_MOTOR_INVERTED);
    roller_motor_.burnFlash();
  }

  @Override
  /**
   * Inside this function, all of the SENSORS should be read into variables stored
   * in the PeriodicIO class defined below. There should be no calls to output to
   * actuators, or any logic within this function.
   */
  public void readPeriodicInputs(double timestamp) {
    if (note_sensor_ != null) {
      io_.note_sensor_range_ = note_sensor_.getRange();
    } else {
      io_.note_sensor_range_ = 1000;
    }
  }

  @Override
  /**
   * Inside this function, all of the LOGIC should compute updates to output
   * variables in the PeriodicIO class defined below. There should be no calls to
   * read from sensors or write to actuators in this function.
   */
  public void updateLogic(double timestamp) {
    if (io_.has_note_pickup_ && io_.note_sensor_range_ > PickupConstants.NO_NOTE_RANGE) {
      io_.has_note_pickup_ = false;
    } else if (io_.has_note_pickup_ == false && io_.note_sensor_range_ < PickupConstants.HAS_NOTE_RANGE) {
      io_.has_note_pickup_ = true;
    }

    switch (io_.pickup_mode_) {
      case PICKUP:
        setRollersForward();
        if (io_.has_note_pickup_) {
          tellShooterReady();
          setIdleMode();
        }
        break;
      case TRANSFER:
        setRollersForward();
        if (io_.has_note_reciever_) {
          setIdleMode();
        }
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
  /**
   * Inside this function actuator OUTPUTS should be updated from data contained
   * in
   * the PeriodicIO class defined below. There should be little to no logic
   * contained within this function, and no sensors should be read.
   */
  public void writePeriodicOutputs(double timestamp) {
    roller_motor_.set(io_.roller_speed_);
  }

  @Override
  /**
   * Inside this function telemetry should be output to smartdashboard. The data
   * should be collected out of the PeriodicIO class instance defined below. There
   * should be no sensor information read in this function nor any outputs made to
   * actuators within this function. Only publish to smartdashboard here.
   */
  public void outputTelemetry(double timestamp) {

  }

  @Override
  public LoggableInputs getLogger() {
    return io;
  }

  public void tellShooterReady() {
  }

  public void setRollersForward() {
    io_.roller_speed_ = PickupConstants.ROLLER_FORWARD;
  }

  public void setRollersBackward() {
    io_.roller_speed_ = PickupConstants.ROLLER_REVERSE;
  }

  public void stopRollers() {
    io_.roller_speed_ = 0.0;
  }

  public void setPickupMode() {
    io_.pickup_mode_ = PickupMode.PICKUP;
  }

  public void setTransferMode() {
    io_.pickup_mode_ = PickupMode.TRANSFER;
  }

  public void setIdleMode() {
    io_.pickup_mode_ = PickupMode.IDLE;
  }

  public void setCleanMode() {
    io_.pickup_mode_ = PickupMode.CLEAN;
  }

  public boolean hasNote() {
    return io_.has_note_pickup_;
  }

  @AutoLog
  public class PeriodicIo extends LogData {
    public boolean has_note_pickup_ = false;
    public boolean has_note_reciever_;
    public double roller_speed_ = 0.0;
    public PickupMode pickup_mode_ = PickupMode.IDLE;
    public double note_sensor_range_;
  }
}
