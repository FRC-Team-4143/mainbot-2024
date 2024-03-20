// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
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

  private CANSparkMax left_climber_motor_;
  private RelativeEncoder climber_encoder_;
  private PIDController rio_climber_controller_;

  public enum ClimbTarget {
    HOME,
    HALF,
    MAX,
    CLIMB
  }

  private Command[] endgame_commands_ = {
      new PresetState(), new EngageState(), new PrepareState(), new ClimbState(), new LockState()
  };

  /**
   * 
   */
  private ClimberPeriodicIo io_;

  private ClimberSubsystem() {
    io_ = new ClimberPeriodicIo();
    left_climber_motor_ = new CANSparkMax(ClimberConstants.LEFT_CLIMBER_MOTOR_ID_, MotorType.kBrushless);
    climber_encoder_ = left_climber_motor_.getEncoder();
    rio_climber_controller_ = new PIDController(ClimberConstants.CLIMBER_CONTROLLER_P, 0, 0);

    reset();
  }

  @Override
  public void reset() {
    left_climber_motor_.setInverted(false);
    left_climber_motor_.setIdleMode(IdleMode.kBrake);
    left_climber_motor_.setSmartCurrentLimit(200);
    left_climber_motor_.burnFlash();
  }

  @Override
  public void readPeriodicInputs(double timestamp) {
    io_.current_height_ = climber_encoder_.getPosition(); // Unknown units
  }

  @Override
  public void updateLogic(double timestamp) {
    if(io_.pid_slot_ == 1){
      rio_climber_controller_.setP(ClimberConstants.WEIGHTED_CLIMBER_CONTROLLER_P);
      io_.controller_ff_ = ClimberConstants.WEIGHTED_CLIMBER_CONTROLLER_FF;
    } else {
      rio_climber_controller_.setP(ClimberConstants.CLIMBER_CONTROLLER_P);
      io_.controller_ff_ = ClimberConstants.CLIMBER_CONTROLLER_FF;
    }
    if(io_.manual_control_){
      io_.winch_speed_ = io_.manual_winch_speed_;
    } else {
      io_.winch_speed_ = rio_climber_controller_.calculate(io_.current_height_, io_.target_height_) + io_.controller_ff_;
    }
  }

  @Override
  public void writePeriodicOutputs(double timestamp) {
    left_climber_motor_.set(io_.winch_speed_);
  }

  @Override
  public void outputTelemetry(double timestamp) {
    SmartDashboard.putNumber("Climber Control/Current Height", io_.current_height_); // Unknown units
    SmartDashboard.putNumber("Climber Control/Target Height", io_.target_height_);
    SmartDashboard.putNumber("Climber Control/Winch Speed", io_.winch_speed_);
    SmartDashboard.putNumber("Climber Control/Motor1 Applied Output", left_climber_motor_.getAppliedOutput());
  }

  public void resetClimberEncoder() {
    climber_encoder_.setPosition(0.0);
  }

  public void setClimbSpeed(double speed) {
    io_.manual_control_ = true;
    io_.manual_winch_speed_ = speed;
  }

  public void stopClimb() {
    io_.manual_winch_speed_ = 0.0;
  }

  public void setHeight(ClimbTarget target, int slot) {
    io_.manual_control_ = false;
    io_.climb_target_ = target;
    if (target == ClimbTarget.MAX) {
      io_.target_height_ = ClimberConstants.MAX_HEIGHT;
    } else if (target == ClimbTarget.HALF) {
      io_.target_height_ = ClimberConstants.HALF_HEIGHT;
    } else if (target == ClimbTarget.CLIMB) {
      io_.target_height_ = ClimberConstants.CLIMB_HEIGHT; 
    } else {
      io_.target_height_ = ClimberConstants.HOME_HEIGHT;
    }
    io_.pid_slot_ = slot;
  }

  public boolean atHeight() {
    return Util.epislonEquals(io_.target_height_, io_.current_height_, ClimberConstants.HEIGHT_TOLERANCE);
  }


  public void raiseHeightTarget() {
    double adjusted = io_.target_height_ - 0.25;
    io_.target_height_ = Math.max(adjusted, ClimberConstants.MAX_HEIGHT);
  }

  public double getTargetHeight() {
    return io_.target_height_;
  }

  public void scheduleNextEndgameState() {
    io_.endgame_state_++;
    io_.endgame_state_ = Math.min(io_.endgame_state_, 4); // Increments the endgame state, up to 4
    endgame_commands_[io_.endgame_state_].schedule();
  }

  public void schedulePreviousEndgameState() {
    io_.endgame_state_--;
    io_.endgame_state_ = Math.max(io_.endgame_state_, 0); // Decrements the endgame state, down to 0
    if (io_.endgame_state_ == 2) {
      new DeclimbState().schedule();
    } else {
      endgame_commands_[io_.endgame_state_].schedule();
    }
  }

  public class ClimberPeriodicIo implements Logged {
    @Log.File
    public double current_height_ = 0.0;
    @Log.File
    public double target_height_ = 0.0;
    @Log.File
    public double winch_speed_ = 0.0;
    @Log.File
    public double controller_ff_ = 0.0;
    @Log.File
    public int endgame_state_ = 0;
    @Log.File
    public int pid_slot_ = 0;
    @Log.File
    public ClimbTarget climb_target_ = ClimbTarget.HOME;
    @Log.File
    public boolean manual_control_ = false;
    @Log.File
    public double manual_winch_speed_ = 0.0;
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
