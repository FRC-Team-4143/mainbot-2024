// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  private CANSparkFlex left_climber_motor_;
  private CANSparkFlex right_climber_motor_;

  /**
   * 
   */
  private ClimberPeriodicIo io_;

  private ClimberSubsystem() {
    left_climber_motor_ = new CANSparkFlex(ClimberConstants.LEFT_CLIMBER_MOTOR_ID_, MotorType.kBrushless);
    right_climber_motor_ = new CANSparkFlex(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID_, MotorType.kBrushless);
    reset();
  }

  @Override
  public void reset() {
    io_ = new ClimberPeriodicIo();
    left_climber_motor_.setInverted(false);
    left_climber_motor_.setIdleMode(IdleMode.kBrake);
    left_climber_motor_.setSmartCurrentLimit(150);

    right_climber_motor_.setInverted(false);
    right_climber_motor_.setIdleMode(IdleMode.kBrake);
    right_climber_motor_.setSmartCurrentLimit(150);
  }

  @Override
  public void readPeriodicInputs(double timestamp) {

  }

  @Override
  public void updateLogic(double timestamp) {

  }

  @Override
  public void writePeriodicOutputs(double timestamp) {
    left_climber_motor_.set(io_.winch_speed_);
    right_climber_motor_.set(io_.winch_speed_);
  }

  @Override
  public void outputTelemetry(double timestamp) {

  }

  public void setClimbSpeed(double speed) {
    io_.winch_speed_ = speed;
  }

  public void stopClimb() {
    io_.winch_speed_ = 0.0;
  }

  public static class ClimberPeriodicIo extends LogData {
    public double winch_speed_ = 0.0;
  }

  @Override
  public LogData getLogger() {
    return io_;
  }
}
