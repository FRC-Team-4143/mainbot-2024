// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.lib.subsystem.Subsystem;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import frc.robot.Constants.PickupConstants;
import frc.robot.Constants.PickupSettings;

public class PickupSubsystem extends Subsystem {

  public enum PickupMode {
    IDLE, PICKUP, TRANSFER
  }

  // Singleton pattern
  private static PickupSubsystem shooterPickupInstance = null;
  private static PickupSubsystem mailmainPickupInstance = null;

  public static PickupSubsystem getShooterInstance() {
    if (shooterPickupInstance == null) {
      shooterPickupInstance = new PickupSubsystem(PickupConstants.shooter_pickup);
    }
    return shooterPickupInstance;
  }

  public static PickupSubsystem getMailmanInstance() {
    if (mailmainPickupInstance == null) {
      mailmainPickupInstance = new PickupSubsystem(PickupConstants.mailman_pickup);
    }
    return mailmainPickupInstance;
  }

  /**
   * 
   */
  private PeriodicIo io;
  private final CANSparkFlex pickup_rollers;
  private final PickupSettings pickup_settings;

  /**
   * Constructor for the example subsystem. The constructor should create all
   * instances of the required hardware as well as the PeriodicIO class defined
   * below. This should not attempt to configure any of the hardware as that
   * should be done in the reset() function.
   */
  private PickupSubsystem(PickupSettings settings) {
    pickup_settings = settings;
    io = new PeriodicIo();
    pickup_rollers = new CANSparkFlex(settings.rollerID, CANSparkLowLevel.MotorType.kBrushless);
    reset();
  }

  @Override
  /**
   * Inside this function should be logic and code to fully reset your subsystem.
   * This is called during initialization, and should handle I/O configuration and
   * initializing data members. pickup_rollers
   */
  public void reset() {
    io = new PeriodicIo();
    pickup_rollers.setSmartCurrentLimit(PickupConstants.rollerAmpLimit);
    pickup_rollers.setInverted(pickup_settings.rollerInverted);
    pickup_rollers.burnFlash();
  }

  @Override
  /**
   * Inside this function, all of the SENSORS should be read into variables stored
   * in the PeriodicIO class defined below. There should be no calls to output to
   * actuators, or any logic within this function.
   */
  public void readPeriodicInputs(double timestamp) {
    // TODO Need reciever subsystem to tell if it has a note and know the note
    // sensor
  }

  @Override
  /**
   * Inside this function, all of the LOGIC should compute updates to output
   * variables in the PeriodicIO class defined below. There should be no calls to
   * read from sensors or write to actuators in this function.
   */
  public void updateLogic(double timestamp) {

    switch (io.pickupMode) {
      case PICKUP:
        setRollersForward();
        if (io.hasNotePickup) {
          tellShooterReady();
          setIdleMode();
        }
        break;
      case TRANSFER:
        setRollersForward();
        if (io.hasNoteReciever) {
          setIdleMode();
        }
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
    pickup_rollers.set(io.rollerSpeed);

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
  public LogData getLogger() {
    return io;
  }

  public void tellShooterReady() {
  }

  public void setRollersForward() {
    io.rollerSpeed = PickupConstants.rollerForward;
  }

  public void setRollersBackward() {
    io.rollerSpeed = PickupConstants.rollerReverse;
  }

  public void stopRollers() {
    io.rollerSpeed = 0.0;
  }

  public void setPickupMode() {
    io.pickupMode = PickupMode.PICKUP;
  }

  public void setTransferMode() {
    io.pickupMode = PickupMode.TRANSFER;
  }

  public void setIdleMode() {
    io.pickupMode = PickupMode.IDLE;
  }

  public class PeriodicIo extends LogData {
    public boolean hasNotePickup = false;
    public boolean hasNoteReciever;
    public double rollerSpeed = 0.0;
    public PickupMode pickupMode = PickupMode.IDLE;
  }
}
