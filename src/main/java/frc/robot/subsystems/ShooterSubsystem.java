// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.Util;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.geometry.Rotation3d;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class ShooterSubsystem extends Subsystem {
    // Singleton pattern
    private static ShooterSubsystem shooterInstance = null;

    public static ShooterSubsystem getInstance() {
        if (shooterInstance == null) {
            shooterInstance = new ShooterSubsystem();
        }
        return shooterInstance;
    }

    // initialize motors
    private CANSparkFlex top_flywheel_motor_;
    private CANSparkFlex bot_flywheel_motor_;
    private CANSparkMax wrist_motor_;
    private CANSparkMax roller_motor_;
    private TimeOfFlight note_sensor_;

    private AprilTagFieldLayout field_layout_ = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // TODO: figure out transformation
    private final Transform3d SPEAKER_TRANSFORM = new Transform3d(0.05, 0, 0.65, new Rotation3d(0, 0, 0)); 
    private final Transform3d AMP_TRANSFORM = new Transform3d(0, 0, -0.5, new Rotation3d(0, 0, 0));

    private final Pose3d BLUE_SPEAKER = field_layout_.getTagPose(7).get().transformBy(SPEAKER_TRANSFORM);
    private final Pose3d RED_SPEAKER = field_layout_.getTagPose(4).get().transformBy(SPEAKER_TRANSFORM)
            .transformBy(new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.PI)));
    private final Pose3d BLUE_AMP = field_layout_.getTagPose(6).get().transformBy(AMP_TRANSFORM);
    private final Pose3d RED_AMP = field_layout_.getTagPose(5).get().transformBy(AMP_TRANSFORM);

    SparkPIDController wrist_controller_;
    SparkAbsoluteEncoder wrist_encoder_;

    private StructPublisher<Pose3d> target_pub;
    private StructPublisher<Pose2d> rot_pub;

    public enum ShootTarget {
        SPEAKER,
        AMP
    }

    public enum ShootMode {
        TARGET,
        IDLE,
        READY,
        TRANSFER,
        CLIMB
    }

    private ShooterPeriodicIoAutoLogged io_;

    public ShooterSubsystem() {
        io_ = new ShooterPeriodicIoAutoLogged();
        top_flywheel_motor_ = new CANSparkFlex(ShooterConstants.TOP_FLYWHEEL_MOTOR_ID,
                CANSparkLowLevel.MotorType.kBrushless);
        bot_flywheel_motor_ = new CANSparkFlex(ShooterConstants.BOT_FLYWHEEL_MOTOR_ID,
                CANSparkLowLevel.MotorType.kBrushless);
        top_flywheel_motor_.follow(bot_flywheel_motor_, true);

        wrist_motor_ = new CANSparkMax(ShooterConstants.WRIST_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        wrist_motor_.setInverted(true);
        wrist_motor_.setIdleMode(IdleMode.kBrake);
        wrist_encoder_ = wrist_motor_.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        wrist_encoder_.setInverted(true);
        wrist_controller_ = wrist_motor_.getPIDController();
        wrist_controller_.setFeedbackDevice(wrist_encoder_);
        wrist_controller_.setP(ShooterConstants.WRIST_CONTROLLER_P);

        roller_motor_ = new CANSparkMax(ShooterConstants.ROLLER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        roller_motor_.setInverted(true);

        target_pub = NetworkTableInstance.getDefault().getStructTopic("tag_pose", Pose3d.struct).publish();
        rot_pub = NetworkTableInstance.getDefault().getStructTopic("rot_pose", Pose2d.struct).publish();

        note_sensor_ = new TimeOfFlight(ShooterConstants.NOTE_SENSOR_ID);
        note_sensor_.setRangingMode(TimeOfFlight.RangingMode.Medium, ShooterConstants.SENSOR_SAMPLE_TIME);
    }

    @Override
    public void reset() {
        io_ = new ShooterPeriodicIoAutoLogged();
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        io_.current_flywheel_speed_ = top_flywheel_motor_.getEncoder().getVelocity();
        io_.current_wrist_angle_ = wrist_encoder_.getPosition() * (2 * Math.PI) - ShooterConstants.WRIST_ZERO_ANGLE;
        io_.note_sensor_range_ = note_sensor_.getRange();

    }

    @Override
    public void updateLogic(double timestamp) {
        Pose2d robot_pose = PoseEstimator.getInstance().getRobotPose();

        if (io_.target_mode_ == ShootMode.TARGET) {
            io_.target_robot_yaw_ = calculateTargetYaw();
            io_.target_wrist_angle_ = calculateWristAngle(robot_pose, io_.target_, calculateNoteExitVelocity());
            io_.relative_chassis_speed_ = transformChassisVelocity();
        } else if (io_.target_mode_ == ShootMode.IDLE) {
            io_.target_wrist_angle_ = ShooterConstants.WRIST_HOME_ANGLE;
            io_.target_flywheel_speed_ = 0;        
        } else if (io_.target_mode_ == ShootMode.TRANSFER){
            io_.target_wrist_angle_ = ShooterConstants.WRIST_HANDOFF_ANGLE;
            io_.target_flywheel_speed_ = 0.1;        
        } else if (io_.target_mode_ == ShootMode.CLIMB) {
            io_.target_wrist_angle_ = ShooterConstants.WRIST_CLIMB_ANGLE;
            io_.target_flywheel_speed_ = 0;           
        }

        hasNote();
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        bot_flywheel_motor_.set(io_.target_flywheel_speed_);
        roller_motor_.set(io_.roller_speed_);
        // setWristAngle();
        SwerveDrivetrain.getInstance().setTargetRotation(io_.target_robot_yaw_);

    }

    @Override
    public void outputTelemetry(double timestamp) {
        SmartDashboard.putNumber("Target Yaw", io_.target_robot_yaw_.getDegrees());
        target_pub.set(io_.target_);
        rot_pub.set(new Pose2d(PoseEstimator.getInstance().getRobotPose().getTranslation(), io_.target_robot_yaw_));
        SmartDashboard.putNumber(" Current Wrist Angle", io_.current_wrist_angle_ * 180 / 3.14159);
        SmartDashboard.putNumber("Target Wrist Angle", io_.target_wrist_angle_ * 180 / 3.14159);
        SmartDashboard.putNumber("Exit Speed", calculateNoteExitVelocity());
        SmartDashboard.putBoolean("Shooter Has Note", io_.has_note_);
        SmartDashboard.putNumber("Shooter Note Sensor Range", io_.note_sensor_range_);
    }

    // get methods
    public double getShooterAngle() {
        return io_.current_wrist_angle_;
    }

    public boolean isTargetLocked() {
        return Util.epislonEquals(io_.current_wrist_angle_, io_.target_wrist_angle_, 
                ShooterConstants.WRIST_TOLERANCE)
                &&
                Util.epislonEquals(io_.current_flywheel_speed_, io_.target_flywheel_speed_,
                ShooterConstants.FLYWHEEL_TOLERANCE)
                &&
                Util.epislonEquals(io_.target_robot_yaw_.getRadians(), PoseEstimator.getInstance().getRobotPose().getRotation().getRadians(), 
                ShooterConstants.YAW_TOLERANCE);
    }

    public void hasNote() {
        if(io_.has_note_ && io_.note_sensor_range_ > ShooterConstants.NO_NOTE_RANGE){
            io_.has_note_ = false;
        } else if(io_.has_note_ == false && io_.note_sensor_range_ < ShooterConstants.HAS_NOTE_RANGE){
            io_.has_note_ = true;
        }
    }

    // set methods
    public void setTarget(ShootTarget target) {
        var alliance = DriverStation.getAlliance();
        if (DriverStation.Alliance.Red == alliance.get()) {
            if (target == ShootTarget.SPEAKER) {
                io_.target_ = RED_SPEAKER;
            } else {
                io_.target_ = RED_AMP;
            }
        } else {
            if (target == ShootTarget.SPEAKER) {
                io_.target_ = BLUE_SPEAKER;
            } else {
                io_.target_ = BLUE_AMP;
            }
        }
    }

    public void setShootMode(ShootMode mode) {
        io_.target_mode_ = mode;
    }

    public void setRollerFeed() {
        io_.roller_speed_ = ShooterConstants.ROLLER_SPEED;
    }

    public void setRollerReverse() {
        io_.roller_speed_ = -ShooterConstants.ROLLER_SPEED;
    }

    public void rollerStop() {
        io_.roller_speed_ = 0;
    }

    // TODO: This method should either be rewritten or only used for manual
    // overrides
    // THIS IS ONLY FOR PROTOTYPE TESTING!!!!
    public void setFlyWheelSpeed(double speed) {
        io_.target_flywheel_speed_ = speed;
    }

    public void flyWheelStop() {
        io_.target_flywheel_speed_ = 0;
    }

    public void setWristAngle() {
        double arb_ff = Math.cos(io_.target_wrist_angle_) * ShooterConstants.WRIST_CONTROLLER_FF;
        wrist_controller_.setReference((io_.target_wrist_angle_ + ShooterConstants.WRIST_ZERO_ANGLE) / (2*Math.PI), ControlType.kPosition, 0, arb_ff);
    }

    public void wristStop() {
        wrist_controller_.setReference(0, ControlType.kVoltage);
    }

    private ChassisSpeeds transformChassisVelocity() {
        ChassisSpeeds temp_chassis_speed = SwerveDrivetrain.getInstance().getCurrentRobotChassisSpeeds();
        io_.target_transform_ = new Transform3d(io_.target_.getTranslation(), io_.target_.getRotation());
        Translation3d temp_translation = new Translation3d(temp_chassis_speed.vxMetersPerSecond,
                temp_chassis_speed.vyMetersPerSecond, 0.0);
        temp_translation.rotateBy(io_.target_transform_.getRotation());
        return new ChassisSpeeds(temp_translation.getX(), temp_translation.getY(), 0.0);
    }

    // Calculate Methods
    public double calculateNoteExitVelocity() {
        return 10;
    }

    private void calculateNoteTravelTime(Pose2d robot_pose, Pose3d target_pose) {
        double distance = (new Pose3d(robot_pose)).getTranslation().getDistance(target_pose.getTranslation());
        io_.note_travel_time_ = distance / ShooterConstants.NOTE_EXIT_VELOCITY; // TODO Find the actual exit velocity
    }

    private double calculateWristAngle(Pose2d robot_pose, Pose3d target_pose, double velocity) {
        Pose3d shooter_pose = (new Pose3d(robot_pose)).transformBy(ShooterConstants.SHOOTER_OFFSET);

        double x = Math.abs(shooter_pose.getX() - target_pose.getX());
        double y = Math.abs(shooter_pose.getY() - target_pose.getY());
        double z = Math.abs(shooter_pose.getZ() - target_pose.getZ());
        double d = Math.sqrt((x * x) + (y * y));
        double G = 9.81;
        double root = Math.pow(velocity, 4) - G * (G * d * d + 2 * velocity * velocity * z);
        double result = Math.atan2((velocity * velocity) - Math.sqrt(root), G * d);
        if (result > 1.5707 || result < 0 || Double.isNaN(result)) {
            return ShooterConstants.WRIST_HOME_ANGLE;
        }
        return result;
    }

    private Rotation2d calculateTargetYaw() {
        Pose2d pose_difference = PoseEstimator.getInstance().getRobotPose().relativeTo(io_.target_.toPose2d());
        return pose_difference.getTranslation().getAngle();
    }


    @AutoLog
    public static class ShooterPeriodicIo extends LogData {
        public Pose3d target_ = new Pose3d();
        public double target_flywheel_speed_ = 0.0;
        public double current_flywheel_speed_ = 0.0;
        public double target_wrist_angle_ = 0.0;
        public double current_wrist_angle_ = 0.0;
        public ShootMode target_mode_ = ShootMode.IDLE;
        public double roller_speed_ = 0.0;
        public boolean has_note_ = false;
        public Rotation2d target_robot_yaw_ = new Rotation2d();
        public double note_travel_time_ = 0.0;
        public Transform3d target_transform_ = new Transform3d();
        public ChassisSpeeds relative_chassis_speed_ = new ChassisSpeeds();
        public double note_sensor_range_ = 0.0;
    }

    @Override
    public LoggableInputs getLogger() {
        return io_;
    }
}
