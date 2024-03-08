// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.subsystem.Subsystem;

import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.climberSequence.ClimbState;
import frc.robot.commands.climberSequence.EngageState;
import frc.robot.commands.climberSequence.LockState;
import frc.robot.commands.climberSequence.PresetState;

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
  private RelativeEncoder climber_encoder_;
  private SparkPIDController climber_controller_;

  public enum ClimbTarget {
    HOME,
    HALF,
    MAX
  }

  private Command[] endgame_commands_ = {
      new PresetState(), new EngageState(), new ClimbState(), new LockState()
  };

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

    climber_encoder_ = left_climber_motor_.getEncoder();
    climber_controller_ = left_climber_motor_.getPIDController();
    climber_controller_.setFeedbackDevice(climber_encoder_);
    climber_controller_.setP(ClimberConstants.CLIMBER_CONTROLLER_P);
    climber_controller_.setD(ClimberConstants.CLIMBER_CONTROLLER_D); // Currently arbitrary values
    right_climber_motor_.follow(left_climber_motor_, false);
    left_climber_motor_.burnFlash();
    right_climber_motor_.burnFlash();

  }

  @Override
  public void readPeriodicInputs(double timestamp) {
    io_.current_height_ = climber_encoder_.getPosition(); // Unknown units
  }

  @Override
  public void updateLogic(double timestamp) {

  }

  @Override
  public void writePeriodicOutputs(double timestamp) {
    left_climber_motor_.set(io_.winch_speed_);
    right_climber_motor_.set(io_.winch_speed_);
    // climber_controller_.setReference(io_.target_height_, ControlType.kPosition,
    // 0,
    // ClimberConstants.CLIMBER_CONTROLLER_FF);
  }

  @Override
  public void outputTelemetry(double timestamp) {
    SmartDashboard.putNumber("Current Climb Height", io_.current_height_); // Unknown units
  }

  public void setClimbSpeed(double speed) {
    io_.winch_speed_ = speed;
  }

  public void stopClimb() {
    io_.winch_speed_ = 0.0;
  }

  public void setHeight(ClimbTarget target) {
    if (target == ClimbTarget.MAX) {
      io_.target_height_ = ClimberConstants.MAX_HEIGHT;
    } else if (target == ClimbTarget.HALF) {
      io_.target_height_ = ClimberConstants.HALF_HEIGHT;
    } else {
      io_.target_height_ = ClimberConstants.HOME_HEIGHT;
    }
  }


  public void getNextEndgameState() {
    io_.endgame_state_++;
    io_.endgame_state_ = Math.min(io_.endgame_state_, 3); // Increments the endgame state, up to 3
    endgame_commands_[io_.endgame_state_].schedule();
  }

  public void getPreviousEndgameState() {
    io_.endgame_state_--;
    io_.endgame_state_ = Math.max(io_.endgame_state_, 0); // Decrements the endgame state, down to 0
    endgame_commands_[io_.endgame_state_].schedule();
  }

  @AutoLog
  public static class ClimberPeriodicIo extends LogData {
    public double current_height_ = 0.0;
    public double target_height_ = 0.0;
    public double winch_speed_ = 0.0;
    public int endgame_state_ = 0;
  }

  @Override
  public LogData getLogger() {
    return io_;
  }
}
