// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Util;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.MailmanConstants;
import monologue.Logged;
import monologue.Annotations.Log;

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
    private PWMSparkMax dropper_motor_;
    private SparkPIDController elevator_controller_;

    //private AprilTagFieldLayout field_layout_ = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    //private final Pose3d AMP = field_layout_.getTagPose(5).get();

    public enum HeightTarget {
        AMP,
        TRAP,
        HOME
    }

    private MailmanSubsystem() {
        io_ = new MailmanPeriodicIo();
        dropper_motor_ = new PWMSparkMax(MailmanConstants.DROPPER_MOTOR_ID);
        elevator_motor_ = new CANSparkMax(MailmanConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        reset();
    }

    @Override
    public void reset() {
        elevator_encoder_ = elevator_motor_.getEncoder();
        elevator_controller_ = elevator_motor_.getPIDController();
        elevator_controller_.setFeedbackDevice(elevator_encoder_);
        elevator_controller_.setP(MailmanConstants.ELEVATOR_CONTROLLER_P);
        elevator_controller_.setSmartMotionMaxVelocity(MailmanConstants.ELEVATOR_CONTROLLER_MAX_VEL, 0);
        elevator_controller_.setSmartMotionMaxAccel(MailmanConstants.ELEVATOR_CONTROLLER_MAX_ACC, 0);
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        io_.current_height_ = elevator_encoder_.getPosition();

    }

    @Override
    public void updateLogic(double timestamp) {
        switch (io_.target_){
            case AMP:
                io_.target_height_ = MailmanConstants.AMP_HEIGHT;
                break;
            case TRAP:
                io_.target_height_ = MailmanConstants.TRAP_HEIGHT;
                break;
            case HOME:
            default:
                io_.target_height_ = MailmanConstants.HOME_HEIGHT;
                break;
        }
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        elevator_controller_.setReference(io_.target_height_, ControlType.kPosition);
        dropper_motor_.set(io_.roller_speed_);
    }

    @Override
    public void outputTelemetry(double timestamp) {
        SmartDashboard.putNumber("Elevator Control/Target Height", io_.target_height_);
        SmartDashboard.putNumber("Elevator Control/Current Height", io_.current_height_);
        SmartDashboard.putNumber("Elevator Control/Applied Output", elevator_motor_.getAppliedOutput()); 
    }

    public boolean atHeight() {
        return Util.epislonEquals(io_.current_height_, io_.target_height_, MailmanConstants.ELEVATOR_HEIGHT_TOLERANCE);
    }

    public HeightTarget getTarget(){
        return io_.target_;
    }

    /**
     * Sets the height to move the mailman to, like HeightTarget.AMP
     * @param target The target height
     * @see HeightTarget
     */
    public void setHeight(HeightTarget target) {
        io_.target_ = target;
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
        io_.roller_speed_ = -0.50;
    }

    public void setRollerBump() {
        io_.roller_speed_ = MailmanConstants.DROPPER_BUMP_SPEED;
    }

    public void setRollerSpeed(double speed){
        io_.roller_speed_ = speed;  
    }

    public void setTargetYaw() {
        SwerveDrivetrain.getInstance().setTargetRotation(Rotation2d.fromDegrees(90).rotateBy(SwerveDrivetrain.getInstance().getDriverPrespective()));
    }

    public void resetElevatorEncoder() {
        elevator_encoder_.setPosition(0);
    }

    public void lowerElevatorHeightTarget() {
        io_.target_height_ -= 0.25;
    }

    public class MailmanPeriodicIo implements Logged {
        @Log.File
        public double current_height_ = 0.0;
        @Log.File
        public double target_height_ = 0.0;
        @Log.File
        public boolean is_allinged_ = false;
        @Log.File
        public double roller_speed_ = 0.0;
        @Log.File
        public Rotation2d target_rotation_ = new Rotation2d();
        @Log.File
        public HeightTarget target_ = HeightTarget.HOME;
    }

    @Override
    public Logged getLoggingObject() {
      return io_;
    }
}
