// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.Util;
import frc.lib.subsystem.Subsystem;

import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.climberSequence.*;
import monologue.Logged;
import monologue.Annotations.Log;

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
  private PIDController rio_climber_controller_;

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
    io_ = new ClimberPeriodicIo();
    left_climber_motor_ = new CANSparkFlex(ClimberConstants.LEFT_CLIMBER_MOTOR_ID_, MotorType.kBrushless);
    right_climber_motor_ = new CANSparkFlex(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID_, MotorType.kBrushless);
    climber_encoder_ = left_climber_motor_.getEncoder();
    //climber_controller_ = left_climber_motor_.getPIDController();
    rio_climber_controller_ = new PIDController(ClimberConstants.CLIMBER_CONTROLLER_P, 0, 0);

    reset();
  }

  @Override
  public void reset() {
    left_climber_motor_.setInverted(false);
    left_climber_motor_.setIdleMode(IdleMode.kBrake);
    left_climber_motor_.setSmartCurrentLimit(200);
    left_climber_motor_.burnFlash();

    right_climber_motor_.setInverted(false);
    right_climber_motor_.setIdleMode(IdleMode.kBrake);
    right_climber_motor_.setSmartCurrentLimit(200);
    //right_climber_motor_.follow(left_climber_motor_, false);
    right_climber_motor_.burnFlash();

    // climber_controller_.setFeedbackDevice(climber_encoder_);
    // climber_controller_.setP(ClimberConstants.CLIMBER_CONTROLLER_P, 0);
    // climber_controller_.setP(ClimberConstants.WEIGHTED_CLIMBER_CONTROLLER_P, 1);
    // climber_controller_.setFF(ClimberConstants.CLIMBER_CONTROLLER_FF, 0);
    // climber_controller_.setFF(ClimberConstants.WEIGHTED_CLIMBER_CONTROLLER_FF, 1);

  }

  @Override
  public void readPeriodicInputs(double timestamp) {
    io_.current_height_ = climber_encoder_.getPosition(); // Unknown units
  }

  @Override
  public void updateLogic(double timestamp) {
    if(io_.pid_slot_ == 1){
      rio_climber_controller_.setP(ClimberConstants.WEIGHTED_CLIMBER_CONTROLLER_P);
    } else {
      rio_climber_controller_.setP(ClimberConstants.CLIMBER_CONTROLLER_P);
    }
    io_.winch_speed_ = rio_climber_controller_.calculate(io_.current_height_, io_.target_height_);
  }

  @Override
  public void writePeriodicOutputs(double timestamp) {
    left_climber_motor_.set(io_.winch_speed_);
    right_climber_motor_.set(io_.winch_speed_);
    // climber_controller_.setReference(io_.target_height_, ControlType.kPosition,
    //     io_.pid_slot_);
  }

  @Override
  public void outputTelemetry(double timestamp) {
    SmartDashboard.putNumber("Current Climb Height", io_.current_height_); // Unknown units
    SmartDashboard.putNumber("Cliber Motor 1 Output", left_climber_motor_.getAppliedOutput());
    SmartDashboard.putNumber("Cliber Motor 2 Output", right_climber_motor_.getAppliedOutput());
  }

  public void resetClimberEncoder() {
    climber_encoder_.setPosition(0.0);
  }

  public void setClimbSpeed(double speed) {
    io_.winch_speed_ = speed;
  }

  public void stopClimb() {
    io_.winch_speed_ = 0.0;
  }

  public void setHeight(ClimbTarget target, int slot) {
    if (target == ClimbTarget.MAX) {
      io_.target_height_ = ClimberConstants.MAX_HEIGHT;
    } else if (target == ClimbTarget.HALF) {
      io_.target_height_ = ClimberConstants.HALF_HEIGHT;
    } else {
      io_.target_height_ = ClimberConstants.HOME_HEIGHT;
    }
    io_.pid_slot_ = slot;
  }

  public boolean atHeight() {
    return Util.epislonEquals(io_.target_height_, io_.current_height_, ClimberConstants.HEIGHT_TOLERANCE);
  }

  public void scheduleNextEndgameState() {
    io_.endgame_state_++;
    io_.endgame_state_ = Math.min(io_.endgame_state_, 3); // Increments the endgame state, up to 3
    endgame_commands_[io_.endgame_state_].schedule();
  }

  public void schedulePreviousEndgameState() {
    io_.endgame_state_--;
    io_.endgame_state_ = Math.max(io_.endgame_state_, 0); // Decrements the endgame state, down to 0
    if (io_.endgame_state_ == 1) {
      new DeclimbState().schedule();
    } else {
      endgame_commands_[io_.endgame_state_].schedule();
    }
  }

  public void raiseHeightTarget() {
    double adjusted = io_.target_height_ - 0.25;
    io_.target_height_ = Math.max(adjusted, ClimberConstants.MAX_HEIGHT);
  }

  public double getTargetHeight() {
    return io_.target_height_;
  }

  public class ClimberPeriodicIo implements Logged {
    @Log.File
    public double current_height_ = 0.0;
    @Log.File
    public double target_height_ = 0.0;
    @Log.File
    public double winch_speed_ = 0.0;
    @Log.File
    public int endgame_state_ = 0;
    @Log.File
    public int pid_slot_ = 0;
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
