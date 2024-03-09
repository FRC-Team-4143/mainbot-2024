// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase.ControlType;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Util;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.MailmanConstants;
import frc.lib.swerve.SwerveRequest;

public class MailmanSubsystem extends Subsystem {

    // Singleton pattern
    private static MailmanSubsystem mailmanInstance = null;

    public static MailmanSubsystem getInstance() {
        if (mailmanInstance == null) {
            mailmanInstance = new MailmanSubsystem();
        }
        return mailmanInstance;
    }

    /**
     * Class Members
     */
    private MailmanPeriodicIo io_;
    private CANSparkMax elevator_motor_;
    private RelativeEncoder elevator_encoder_;
    private CANSparkFlex dropper_motor_;
    private SparkPIDController elevator_controller_;
    private TimeOfFlight note_sensor_;


    public enum HeightTarget {
        AMP,
        TRAP,
        HOME
    }

    private MailmanSubsystem() {
        elevator_motor_ = new CANSparkMax(MailmanConstants.ELEVATOR_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        dropper_motor_ = new CANSparkFlex(MailmanConstants.DROPPER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        note_sensor_ = new TimeOfFlight(MailmanConstants.NOTE_SENSOR_ID);
        note_sensor_.setRangingMode(TimeOfFlight.RangingMode.Medium, MailmanConstants.SENSOR_SAMPLE_TIME);
        reset();
    }

    @Override
    public void reset() {
        io_ = new MailmanPeriodicIo();

        elevator_encoder_ = elevator_motor_.getEncoder();
        elevator_controller_ = elevator_motor_.getPIDController();
        elevator_controller_.setFeedbackDevice(elevator_encoder_);
        elevator_controller_.setP(MailmanConstants.ELEVATOR_CONTROLLER_P);
        elevator_controller_.setSmartMotionMaxVelocity(MailmanConstants.ELEVATOR_CONTROLLER_MAX_VEL, 0);
        elevator_controller_.setSmartMotionMaxAccel(MailmanConstants.ELEVATOR_CONTROLLER_MAX_ACC, 0);
        dropper_motor_.setSmartCurrentLimit(80);
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        io_.current_height_ = elevator_encoder_.getPosition();
        io_.note_sensor_range_ = note_sensor_.getRange();

    }

    @Override
    public void updateLogic(double timestamp) {
        if (io_.has_note_ && io_.note_sensor_range_ > MailmanConstants.NO_NOTE_RANGE) {
            io_.has_note_ = false;
        } else if (io_.has_note_ == false && io_.note_sensor_range_ < MailmanConstants.HAS_NOTE_RANGE) {
            io_.has_note_ = true;
        }
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        elevator_controller_.setReference(io_.target_height_, ControlType.kPosition);
        dropper_motor_.set(io_.roller_speed_);
    }

    @Override
    public void outputTelemetry(double timestamp) {
        SmartDashboard.putNumber("Current Elevator Height", io_.current_height_);
        SmartDashboard.putNumber("Elevator Applied Output", elevator_motor_.getAppliedOutput());
    }

    public boolean atHeight() {
        return Util.epislonEquals(io_.current_height_, io_.target_height_);
    }

    /**
     * Sets the height to move the mailman to, like HeightTarget.AMP
     * @param target The target height
     * @see HeightTarget
     */
    public void setHeight(HeightTarget target) {
        if (target == HeightTarget.AMP) {
            io_.target_height_ = MailmanConstants.AMP_HEIGHT;
        } else if (target == HeightTarget.TRAP) {
            io_.target_height_ = MailmanConstants.TRAP_HEIGHT;
        } else {
            io_.target_height_ = MailmanConstants.HOME_HEIGHT;
        }
    }

    /**
     * Turns the rollers on
     */
    public void setRollerIntake() {
        io_.roller_speed_ = MailmanConstants.DROPPER_IN_SPEED;
    }

    /**
     * Reverses the rollers
     */
    public void setRollerOutput() {
        io_.roller_speed_ = MailmanConstants.DROPPER_OUT_SPEED;
    }

    /**
     * Stops the rollers
     */
    public void setRollerStop() {
        io_.roller_speed_ = 0;
    }

    /**
     * Runs the rollers slowly, to recieve from the shooter handoff
     */
    public void setRollerRecieve() {
        io_.roller_speed_ = -0.15;
    }

    public void lineupToAmp(){
    }

    public static class MailmanPeriodicIo extends LogData {
        public double current_height_ = 0.0;
        public double target_height_ = 0.0;
        public boolean is_holding_note_ = false;
        public boolean is_allinged_ = false;
        public boolean note_wanted_elsewhere_ = false;
        public double roller_speed_ = 0.0;
        public boolean has_note_ = false;
        public double note_sensor_range_= 0.0;
    }

    @Override
    public LogData getLogger() {
        return io_;
    }
}
