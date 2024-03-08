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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.geometry.Rotation3d;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

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

    private SparkPIDController wrist_controller_;
    private SparkAbsoluteEncoder wrist_encoder_;
    private SparkPIDController top_flywheel_controller_;
    private RelativeEncoder top_flywheel_encoder_;
    private SparkPIDController bot_flywheel_controller_;
    private RelativeEncoder bot_flywheel_encoder_;

    private AprilTagFieldLayout field_layout_ = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private final Transform3d SPEAKER_TRANSFORM = new Transform3d(0, 0, 0.65, new Rotation3d(0, 0, 0));
    private final Transform3d AMP_TRANSFORM = new Transform3d(0, 0, -0.5, new Rotation3d(0, 0, 0));
    private final Transform3d RED_PASS_TRANSFORM = new Transform3d(1, -1.25, -1.5, new Rotation3d(0, 0, 0));
    private final Transform3d BLUE_PASS_TRANSFORM = new Transform3d(-1, -1.25, -1.5, new Rotation3d(0, 0, 0));

    // Target positions
    private final Pose3d BLUE_SPEAKER = field_layout_.getTagPose(7).get().transformBy(SPEAKER_TRANSFORM);
    private final Pose3d RED_SPEAKER = field_layout_.getTagPose(4).get().transformBy(SPEAKER_TRANSFORM);
    private final Pose3d BLUE_AMP = field_layout_.getTagPose(6).get().transformBy(AMP_TRANSFORM);
    private final Pose3d RED_AMP = field_layout_.getTagPose(5).get().transformBy(AMP_TRANSFORM);
    private final Pose3d BLUE_PASS = field_layout_.getTagPose(7).get().transformBy(BLUE_PASS_TRANSFORM);
    private final Pose3d RED_PASS = field_layout_.getTagPose(4).get().transformBy(RED_PASS_TRANSFORM);
    // Speed maps
    private final InterpolatingDoubleTreeMap dist_to_angle_offset_lookup_ = ShooterConstants
            .DISTANCE_TO_TARGET_OFFSET_MAP();

    private StructPublisher<Pose3d> target_pub;
    private StructPublisher<Pose2d> rot_pub;

    public enum ShootTarget {
        SPEAKER,
        AMP,
        PASS
    }

    public enum ShootMode {
        TARGET,
        IDLE,
        READY,
        TRANSFER,
        RECEIVE,
        CLIMB,
        PROFILE,
        PASS
    }

    private ShooterPeriodicIo io_;

    public ShooterSubsystem() {
        top_flywheel_motor_ = new CANSparkFlex(ShooterConstants.TOP_FLYWHEEL_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        bot_flywheel_motor_ = new CANSparkFlex(ShooterConstants.BOT_FLYWHEEL_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        wrist_motor_ = new CANSparkMax(ShooterConstants.WRIST_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        roller_motor_ = new CANSparkMax(ShooterConstants.ROLLER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        note_sensor_ = new TimeOfFlight(ShooterConstants.NOTE_SENSOR_ID);
        note_sensor_.setRangingMode(TimeOfFlight.RangingMode.Medium, ShooterConstants.SENSOR_SAMPLE_TIME);

        target_pub = NetworkTableInstance.getDefault().getStructTopic("tag_pose", Pose3d.struct).publish();
        rot_pub = NetworkTableInstance.getDefault().getStructTopic("rot_pose", Pose2d.struct).publish();
    }

    @Override
    public void reset() {
        io_ = new ShooterPeriodicIo();

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
        roller_motor_.setInverted(ShooterConstants.ROLLER_MOTOR_INVERTED);
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
        io_.note_travel_time_ = calculateNoteTravelTime(robot_pose, io_.target_offset_pose);
        io_.relative_chassis_speed_ = transformChassisVelocity();
        io_.target_offset_pose = io_.target_.transformBy(calculateMovingTargetOffset(io_.relative_chassis_speed_, io_.note_travel_time_));
        io_.target_distance_ = calculateLinearDist(robot_pose, io_.target_offset_pose);
        io_.target_offset_lookup_ = dist_to_angle_offset_lookup_.get(io_.target_distance_);

        if (io_.target_mode_ == ShootMode.TARGET) {
            io_.target_robot_yaw_ = calculateTargetYaw(robot_pose, io_.target_offset_pose);
            io_.target_wrist_angle_ = calculateWristAngle(robot_pose, io_.target_offset_pose,
                    ShooterConstants.NOTE_EXIT_VELOCITY);
            io_.target_flywheel_speed_ = 550; // TODO: set ideal shooter rads/s
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
            io_.target_flywheel_speed_ = 550;
        }else if(io_.target_mode_ == ShootMode.PASS){
            io_.target_robot_yaw_ = calculateTargetYaw(robot_pose, io_.target_offset_pose);
            io_.target_wrist_angle_ = Math.toRadians(40);
            //io_.target_wrist_angle_ = calculateWristAnglePass(robot_pose, io_.target_offset_pose, ShooterConstants.NOTE_EXIT_VELOCITY_PASSING);
            io_.target_flywheel_speed_ = 325;
        }

        if (io_.has_note_ && io_.note_sensor_range_ > ShooterConstants.NO_NOTE_RANGE) {
            io_.has_note_ = false;
        } else if (io_.has_note_ == false && io_.note_sensor_range_ < ShooterConstants.HAS_NOTE_RANGE) {
            io_.has_note_ = true;
        }
    }


    @Override
    public void writePeriodicOutputs(double timestamp) {
        roller_motor_.set(io_.roller_speed_);
        setFlyWheelRPM(io_.target_flywheel_speed_);
        setWristAngle(io_.target_wrist_angle_);
        SwerveDrivetrain.getInstance().setTargetRotation(io_.target_robot_yaw_);
    }

    @Override
    public void outputTelemetry(double timestamp) {
        SmartDashboard.putNumber("Target Yaw", io_.target_robot_yaw_.getDegrees());
        target_pub.set(io_.target_offset_pose);
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

        SmartDashboard.putNumber("Wrist Encoder Value", wrist_encoder_.getPosition());

    }

    /**
     * Returns true if the note sensor sees the note in the shooter
     * @return If the shooter has a note
     */
     public boolean hasNote() {
        return io_.has_note_;
    }

    /**
     * Gets the angle of the shooter, in radians. The home position is approximately
     * 0 radians.
     * 
     * @return The angle of the shooter, in radians
     */
    public double getShooterAngle() {
        return io_.current_wrist_angle_;
    }

    /**
     * Returns true if the robot is locked onto the target and ready to shoot. This
     * method will return true if the flywheel speed, shooter angle, and robot yaw
     * is all correct within a certain tolerance.
     * 
     * @return True if the target is locked, otherwise return false.
     */
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
                        SwerveDrivetrain.getInstance().getRobotRotation().rotateBy(SwerveDrivetrain.getInstance().getDriverPrespective()),
                        ShooterConstants.YAW_TOLERANCE);
    }

    /**
     * Returns true if the robot is locked onto the target and ready to shoot. This
     * method will return true if the flywheel speed, shooter angle
     * is all correct within a certain tolerance.
     * 
     * Ignores the Yaw Lineup for Loss of Vision
     * 
     * @return True if the target is locked, otherwise return false.
     */
    public boolean isOverrideTargetLocked() {

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
                        ShooterConstants.FLYWHEEL_TOLERANCE);
    }

    /**
     * Sets the targeting mode to the speaker or the amp.
     * @param target Target to shoot at, either ShootTarget.SPEAKER or ShootTarget.AMP
     * @see ShootTarget
     */
    public void setTarget(ShootTarget target) {
        var alliance = DriverStation.getAlliance();
        if (DriverStation.Alliance.Red == alliance.get()) {
            if (target == ShootTarget.SPEAKER) {
                io_.target_ = RED_SPEAKER;
            } else if(target ==ShootTarget.AMP){
                io_.target_ = RED_AMP;
            }else{
                io_.target_ = RED_PASS;
            }
            
        } else {
            if (target == ShootTarget.SPEAKER) {
                io_.target_ = BLUE_SPEAKER;
            } else if(target ==ShootTarget.AMP){
                io_.target_ = BLUE_AMP;
            }else{
                io_.target_ = BLUE_PASS;
            }
        }
    }

    /**
     * Sets the mode of the shooter subsystem enum, like ShootMode.TARGET
     * @see ShootMode
     * @param mode The mode to change the ShooterSubsystem to
     */
    public void setShootMode(ShootMode mode) {
        io_.target_mode_ = mode;
    }

    /**
     * Turns the rollers on
     */
    public void setRollerFeed() {
        io_.roller_speed_ = ShooterConstants.ROLLER_SPEED;
    }

    /**
     * Reverses the rollers
     */
    public void setRollerReverse() {
        io_.roller_speed_ = -ShooterConstants.ROLLER_SPEED;
    }

    /**
     * Stops the rollers
     */
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

    /**
     * Stops both of the the flywheels
     */
    public void flyWheelStop() {
        top_flywheel_controller_.setReference(0, ControlType.kVelocity, 1);
        bot_flywheel_controller_.setReference(0, ControlType.kVelocity, 1);
    }

    public void setWristAngle(double angle) {
        double arb_ff = Math.cos(angle) * ShooterConstants.WRIST_CONTROLLER_FF;
        wrist_controller_.setReference((angle + ShooterConstants.WRIST_ZERO_ANGLE) / (2 * Math.PI),
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

    private double calculateNoteTravelTime(Pose2d robot_pose, Pose3d target_pose) {
        double distance = (new Pose3d(robot_pose)).getTranslation().getDistance(target_pose.getTranslation());
        return distance / ShooterConstants.NOTE_EXIT_VELOCITY;
    }

    private double calculateLinearDist(Pose2d robot_pose, Pose3d target_pose) {
        Pose3d shooter_pose = (new Pose3d(robot_pose)).transformBy(ShooterConstants.SHOOTER_OFFSET);
        double x = Math.abs(shooter_pose.getX() - io_.target_.getX());
        double y = Math.abs(shooter_pose.getY() - io_.target_.getY());
        return Math.sqrt((x * x) + (y * y));
    }

    public double getLinearDist(){
        return io_.target_distance_;
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

    private double calculateWristAnglePass(Pose2d robot_pose, Pose3d target_pose, double velocity) {
        Pose3d shooter_pose = (new Pose3d(robot_pose)).transformBy(ShooterConstants.SHOOTER_OFFSET);

        double z = Math.abs(shooter_pose.getZ() - target_pose.getZ()) + io_.target_offset_lookup_;
        double d = calculateLinearDist(robot_pose, target_pose);
        double G = 9.81;
        double root = Math.pow(velocity, 4) - G * (G * d * d + 2 * velocity * velocity * z);
        double result = Math.atan2((velocity * velocity) + Math.sqrt(root), G * d);
        if (result > 1.5707 || result < 0 || Double.isNaN(result)) {
            return ShooterConstants.WRIST_HOME_ANGLE;
        }
        return result;
    }

    private Rotation2d calculateTargetYaw(Pose2d robot_pose, Pose3d target_pose) {
        Pose2d pose_difference = robot_pose.relativeTo(target_pose.toPose2d());
        return pose_difference.getTranslation().getAngle().rotateBy(Rotation2d.fromDegrees(180));
    }

    private Transform3d calculateMovingTargetOffset(ChassisSpeeds chassis_speeds, double travel_time) {
        double depth_offset = -chassis_speeds.vyMetersPerSecond * travel_time;
        double horizontal_offset = -chassis_speeds.vxMetersPerSecond * travel_time;
        return new Transform3d(new Translation3d(horizontal_offset, depth_offset, 0), new Rotation3d());
    }

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
        public Pose3d target_offset_pose = new Pose3d();
    }

    @Override
    public LogData getLogger() {
        return io_;
    }

}
