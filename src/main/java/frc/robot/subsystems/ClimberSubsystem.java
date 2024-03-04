// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.subsystem.Subsystem;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends Subsystem {

  // Singleton pattern
  private static ClimberSubsystem climberInstance = null;

  public static ClimberSubsystem getInstance() {
    if (climberInstance == null) {
      climberInstance = new ClimberSubsystem();
    }
    return climberInstance;
  }

  private final CANSparkFlex left_climber_motor_;
  private final CANSparkFlex right_climber_motor_;

  /**
   * 
   */
  private ClimberPeriodicIoAutoLogged io_;

  private ClimberSubsystem() {
    io_ = new ClimberPeriodicIoAutoLogged();
    left_climber_motor_ = new CANSparkFlex(ClimberConstants.LEFT_CLIMBER_MOTOR_ID_, MotorType.kBrushless);
    left_climber_motor_.setInverted(false);
    left_climber_motor_.setIdleMode(IdleMode.kBrake);
    left_climber_motor_.setSmartCurrentLimit(150);

    right_climber_motor_ = new CANSparkFlex(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID_, MotorType.kBrushless);
    right_climber_motor_.setInverted(false);
    right_climber_motor_.setIdleMode(IdleMode.kBrake);
    right_climber_motor_.setSmartCurrentLimit(150);
  }

  public void setClimbSpeed(double speed) {
    io_.winch_speed_ = speed;
  }

  public void stopClimb() {
    io_.winch_speed_ = 0.0;
  }

  @Override
  /**
   * Inside this function should be logic and code to fully reset your subsystem.
   * This is called during initialization, and should handle I/O configuration and
   * initializing data members.
   */
  public void reset() {
    io_ = new ClimberPeriodicIoAutoLogged();
  }

  @Override
  /**
   * Inside this function, all of the SENSORS should be read into variables stored
   * in the PeriodicIO class defined below. There should be no calls to output to
   * actuators, or any logic within this function.
   */
  public void readPeriodicInputs(double timestamp) {

  }

  @Override
  /**
   * Inside this function, all of the LOGIC should compute updates to output
   * variables in the PeriodicIO class defined below. There should be no calls to
   * read from sensors or write to actuators in this function.
   */
  public void updateLogic(double timestamp) {

  }

  @Override
  /**
   * Inside this function actuator OUTPUTS should be updated from data contained
   * in
   * the PeriodicIO class defined below. There should be little to no logic
   * contained within this function, and no sensors should be read.
   */
  public void writePeriodicOutputs(double timestamp) {
    left_climber_motor_.set(io_.winch_speed_);
    right_climber_motor_.set(io_.winch_speed_);
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
    return io_;
  }

  @AutoLog
  public static class ClimberPeriodicIo extends LogData {
    public double winch_speed_ = 0.0;
  }
}
