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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;

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

    public static synchronized ShooterSubsystem getInstance() {
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
    private final Transform3d SPEAKER_TRANSFORM = new Transform3d(0, 0, 0.65, new Rotation3d(0, 0, 0));
    private final Transform3d AMP_TRANSFORM = new Transform3d(0, 0, -0.5, new Rotation3d(0, 0, 0));

    // Target positions
    private final Pose3d BLUE_SPEAKER = field_layout_.getTagPose(7).get().transformBy(SPEAKER_TRANSFORM);
    private final Pose3d RED_SPEAKER = field_layout_.getTagPose(4).get().transformBy(SPEAKER_TRANSFORM)
            .transformBy(new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.PI)));
    private final Pose3d BLUE_AMP = field_layout_.getTagPose(6).get().transformBy(AMP_TRANSFORM);
    private final Pose3d RED_AMP = field_layout_.getTagPose(5).get().transformBy(AMP_TRANSFORM);

    // Speed maps
    // private final InterpolatingDoubleTreeMap linear_to_angular_vel_map =
    // ShooterConstants.LINEAR_TO_ANGULAR_VEL_MAP();
    // private final InterpolatingDoubleTreeMap distance_to_linear_vel_map =
    // ShooterConstants.DISTANCE_TO_EXIT_VEL_MAP();

    SparkPIDController wrist_controller_;
    SparkAbsoluteEncoder wrist_encoder_;

    SparkPIDController top_flywheel_controller_;
    RelativeEncoder top_flywheel_encoder_;

    SparkPIDController bot_flywheel_controller_;
    RelativeEncoder bot_flywheel_encoder_;

    private StructPublisher<Pose3d> target_pub;
    private StructPublisher<Pose2d> rot_pub;

    private final InterpolatingDoubleTreeMap dist_to_angle_offset_lookup_ = ShooterConstants
            .DISTANCE_TO_TARGET_OFFSET_MAP();

    public enum ShootTarget {
        SPEAKER,
        AMP
    }

    public enum ShootMode {
        TARGET,
        IDLE,
        READY,
        TRANSFER,
        RECEIVE,
        CLIMB,
        PROFILE
    }

    private ShooterPeriodicIoAutoLogged io_;

    public ShooterSubsystem() {
        io_ = new ShooterPeriodicIoAutoLogged();
        
        top_flywheel_motor_ = new CANSparkFlex(ShooterConstants.TOP_FLYWHEEL_MOTOR_ID,
                CANSparkLowLevel.MotorType.kBrushless);
        bot_flywheel_motor_ = new CANSparkFlex(ShooterConstants.BOT_FLYWHEEL_MOTOR_ID,
                CANSparkLowLevel.MotorType.kBrushless);
        

        wrist_motor_ = new CANSparkMax(ShooterConstants.WRIST_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        
        roller_motor_ = new CANSparkMax(ShooterConstants.ROLLER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

        note_sensor_ = new TimeOfFlight(ShooterConstants.NOTE_SENSOR_ID);
        note_sensor_.setRangingMode(TimeOfFlight.RangingMode.Medium, ShooterConstants.SENSOR_SAMPLE_TIME);

        target_pub = NetworkTableInstance.getDefault().getStructTopic("tag_pose", Pose3d.struct).publish();
        rot_pub = NetworkTableInstance.getDefault().getStructTopic("rot_pose", Pose2d.struct).publish();
        
    }

    @Override
    public void reset() {
        io_ = new ShooterPeriodicIoAutoLogged();

        // Top flywheel motor configuration
        top_flywheel_motor_.setSmartCurrentLimit(40);
        top_flywheel_motor_.setInverted(true);
        top_flywheel_controller_ = top_flywheel_motor_.getPIDController();
        top_flywheel_encoder_ = top_flywheel_motor_.getEncoder();
        top_flywheel_controller_.setFeedbackDevice(top_flywheel_encoder_);
        top_flywheel_controller_.setP(ShooterConstants.FLYWHEEL_CONTROLLER_P, 0);
        top_flywheel_controller_.setFF(ShooterConstants.FLYWHEEL_CONTROLLER_FF, 0);
        top_flywheel_controller_.setP(0.0001, 1);
        top_flywheel_controller_.setFF(0, 1);
        top_flywheel_motor_.burnFlash();

        // Bottom flywheel motor configuration
        bot_flywheel_motor_.setSmartCurrentLimit(40);
        bot_flywheel_motor_.setInverted(false);
        bot_flywheel_controller_ = bot_flywheel_motor_.getPIDController();
        bot_flywheel_encoder_ = bot_flywheel_motor_.getEncoder();
        bot_flywheel_controller_.setFeedbackDevice(bot_flywheel_encoder_);
        bot_flywheel_controller_.setP(ShooterConstants.FLYWHEEL_CONTROLLER_P, 0);
        bot_flywheel_controller_.setFF(ShooterConstants.FLYWHEEL_CONTROLLER_FF, 0);
        bot_flywheel_controller_.setP(0.0001, 1);
        bot_flywheel_controller_.setFF(0, 1);
        bot_flywheel_motor_.burnFlash();

        // Wrist motor configuration
        wrist_motor_.setInverted(true);
        wrist_motor_.setIdleMode(IdleMode.kBrake);
        wrist_encoder_ = wrist_motor_.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        wrist_encoder_.setInverted(true);
        wrist_controller_ = wrist_motor_.getPIDController();
        wrist_controller_.setFeedbackDevice(wrist_encoder_);
        wrist_controller_.setP(ShooterConstants.WRIST_CONTROLLER_P);
        wrist_motor_.burnFlash();

        // Roller motor configuration
        roller_motor_.setInverted(true);
        roller_motor_.burnFlash();

    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        io_.current_top_flywheel_speed_ = top_flywheel_encoder_.getVelocity() / 9.5492;
        io_.current_bot_flywheel_speed_ = bot_flywheel_encoder_.getVelocity() / 9.5492;
        io_.current_wrist_angle_ = wrist_encoder_.getPosition() * (2 * Math.PI) - ShooterConstants.WRIST_ZERO_ANGLE;
        io_.note_sensor_range_ = note_sensor_.getRange();

    }

    @Override
    public void updateLogic(double timestamp) {
        Pose2d robot_pose = PoseEstimator.getInstance().getRobotPose();
        io_.target_distance_ = calculateLinearDist(robot_pose, io_.target_);
        io_.target_offset_lookup_ = dist_to_angle_offset_lookup_.get(io_.target_distance_);

        if (io_.target_mode_ == ShootMode.TARGET) {
            io_.target_robot_yaw_ = calculateTargetYaw(robot_pose, io_.target_);
            io_.target_wrist_angle_ = calculateWristAngle(robot_pose, io_.target_, ShooterConstants.NOTE_EXIT_VELOCITY);
            io_.target_flywheel_speed_ = 550; // TODO: set ideal shooter rads/s
            io_.relative_chassis_speed_ = transformChassisVelocity();
        } else if (io_.target_mode_ == ShootMode.IDLE) {
            io_.target_wrist_angle_ = ShooterConstants.WRIST_HOME_ANGLE;
            io_.target_flywheel_speed_ = 0;
        } else if (io_.target_mode_ == ShootMode.TRANSFER) {
            io_.target_wrist_angle_ = ShooterConstants.WRIST_HANDOFF_ANGLE;
            io_.target_flywheel_speed_ = 50;
        } else if (io_.target_mode_ == ShootMode.CLIMB) {
            io_.target_wrist_angle_ = ShooterConstants.WRIST_CLIMB_ANGLE;
            io_.target_flywheel_speed_ = 0;
        } else if (io_.target_mode_ == ShootMode.RECEIVE) {
            io_.target_wrist_angle_ = ShooterConstants.WRIST_HANDOFF_ANGLE;
            io_.target_flywheel_speed_ = -50;
        } else if (io_.target_mode_ == ShootMode.PROFILE) {
            io_.target_wrist_angle_ = Math.toRadians(35);
            io_.target_flywheel_speed_ = 580;
        }

        if (io_.has_note_ && io_.note_sensor_range_ > ShooterConstants.NO_NOTE_RANGE) {
            io_.has_note_ = false;
        } else if (io_.has_note_ == false && io_.note_sensor_range_ < ShooterConstants.HAS_NOTE_RANGE) {
            io_.has_note_ = true;
        }
    }

    public boolean hasNote() {
        return io_.has_note_;
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        roller_motor_.set(io_.roller_speed_);
        setFlyWheelRPM(io_.target_flywheel_speed_);
        setWristAngle();
        SwerveDrivetrain.getInstance().setTargetRotation(io_.target_robot_yaw_);

    }

    @Override
    public void outputTelemetry(double timestamp) {
        SmartDashboard.putNumber("Target Yaw", io_.target_robot_yaw_.getDegrees());
        target_pub.set(io_.target_);
        rot_pub.set(new Pose2d(PoseEstimator.getInstance().getRobotPose().getTranslation(), io_.target_robot_yaw_));

        SmartDashboard.putNumber("Exit Speed", calculateNoteExitVelocity());
        SmartDashboard.putBoolean("Shooter Has Note", io_.has_note_);
        SmartDashboard.putNumber("Shooter Note Sensor Range", io_.note_sensor_range_);

        // Target Locked Tests
        SmartDashboard.putBoolean("Target Locked", this.isTargetLocked());

        SmartDashboard.putNumber("Target Flywheel Speed", io_.target_flywheel_speed_); // * 9.549);
        SmartDashboard.putNumber("Current Top Flywheel Speed", io_.current_top_flywheel_speed_); // * 9.549);
        SmartDashboard.putNumber("Current Bot Flywheel Speed", io_.current_bot_flywheel_speed_); // * 9.549);

        SmartDashboard.putNumber("Target Wrist Angle", io_.target_wrist_angle_); // * 180 / 3.14159);
        SmartDashboard.putNumber("Current Wrist Angle", io_.current_wrist_angle_); // * 180 / 3.14159);

        SmartDashboard.putNumber("Target Robot Yaw", io_.target_robot_yaw_.getRadians());
        SmartDashboard.putNumber("Current Robot Yaw",
                PoseEstimator.getInstance().getRobotPose().getRotation().getRadians());
        SmartDashboard.putNumber("Target Distance", io_.target_distance_);

    }

    // get methods
    public double getShooterAngle() {
        return io_.current_wrist_angle_;
    }

    public boolean isTargetLocked() {

        if (io_.target_flywheel_speed_ == 0) {
            return false;
        }

        return Util.epislonEquals(io_.current_wrist_angle_, io_.target_wrist_angle_,
                ShooterConstants.WRIST_TOLERANCE)
                &&
                Util.epislonEquals(io_.current_top_flywheel_speed_,
                        io_.target_flywheel_speed_,
                        ShooterConstants.FLYWHEEL_TOLERANCE)
                &&
                Util.epislonEquals(io_.current_bot_flywheel_speed_,
                        io_.target_flywheel_speed_,
                        ShooterConstants.FLYWHEEL_TOLERANCE)
                &&
                Util.epislonEquals(io_.target_robot_yaw_,
                        SwerveDrivetrain.getInstance().getRobotRotation(),
                        ShooterConstants.YAW_TOLERANCE);
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

    public void setFlyWheelRPM(double rad_per_sec_) {
        double rpm = rad_per_sec_ * 9.549;
        if (rpm == 0) {
            flyWheelStop();
        } else {
            top_flywheel_controller_.setReference(rpm, ControlType.kVelocity, 0);
            bot_flywheel_controller_.setReference(rpm, ControlType.kVelocity, 0);
        }
    }

    public void flyWheelStop() {
        top_flywheel_controller_.setReference(0, ControlType.kVelocity, 1);
        bot_flywheel_controller_.setReference(0, ControlType.kVelocity, 1);
    }

    public void setWristAngle() {
        double arb_ff = Math.cos(io_.target_wrist_angle_) * ShooterConstants.WRIST_CONTROLLER_FF;
        wrist_controller_.setReference((io_.target_wrist_angle_ + ShooterConstants.WRIST_ZERO_ANGLE) / (2 * Math.PI),
                ControlType.kPosition, 0, arb_ff);
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
        return ShooterConstants.NOTE_EXIT_VELOCITY;
    }

    private void calculateNoteTravelTime(Pose2d robot_pose, Pose3d target_pose) {
        double distance = (new Pose3d(robot_pose)).getTranslation().getDistance(target_pose.getTranslation());
        io_.note_travel_time_ = distance / ShooterConstants.NOTE_EXIT_VELOCITY; // TODO Find the actual exit velocity
    }

    private double calculateLinearDist(Pose2d robot_pose, Pose3d target_pose) {
        Pose3d shooter_pose = (new Pose3d(robot_pose)).transformBy(ShooterConstants.SHOOTER_OFFSET);
        double x = Math.abs(shooter_pose.getX() - io_.target_.getX());
        double y = Math.abs(shooter_pose.getY() - io_.target_.getY());
        return Math.sqrt((x * x) + (y * y));
    }

    private double calculateWristAngle(Pose2d robot_pose, Pose3d target_pose, double velocity) {
        Pose3d shooter_pose = (new Pose3d(robot_pose)).transformBy(ShooterConstants.SHOOTER_OFFSET);

        double z = Math.abs(shooter_pose.getZ() - target_pose.getZ()) + io_.target_offset_lookup_;
        double d = calculateLinearDist(robot_pose, target_pose);
        double G = 9.81;
        double root = Math.pow(velocity, 4) - G * (G * d * d + 2 * velocity * velocity * z);
        double result = Math.atan2((velocity * velocity) - Math.sqrt(root), G * d);
        if (result > 1.5707 || result < 0 || Double.isNaN(result)) {
            return ShooterConstants.WRIST_HOME_ANGLE;
        }
        return result;
    }

    private Rotation2d calculateTargetYaw(Pose2d robot_pose, Pose3d target) {
        Pose2d pose_difference = robot_pose.relativeTo(target.toPose2d());
        return pose_difference.getTranslation().getAngle();
    }

    @AutoLog
    public static class ShooterPeriodicIo extends LogData {
        public Pose3d target_ = new Pose3d();
        public double target_flywheel_speed_ = 0.0;
        public double current_top_flywheel_speed_ = 0.0;
        public double current_bot_flywheel_speed_ = 0.0;
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
        public double target_offset_lookup_ = 0.0;
        public double target_distance_ = 0.0;
    }

    @Override
    public LoggableInputs getLogger() {
        return io_;
    }

}
