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
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class ShooterSubsystem extends Subsystem {
    // Singleton pattern
    private static ShooterSubsystem ShooterInstance = null;

    public static ShooterSubsystem getInstance() {
        if (ShooterInstance == null) {
            ShooterInstance = new ShooterSubsystem();
        }
        return ShooterInstance;
    }

    // initialize motors
    private CANSparkFlex top_flywheel_motor_;
    private CANSparkFlex bot_flywheel_motor_;
    private CANSparkMax wrist_motor_;
    private CANSparkMax roller_motor_;
    private TimeOfFlight note_sensor_;


    private AprilTagFieldLayout field_layout_ = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private final Transform3d SPEAKER_TRANSFORM = new Transform3d(0.05, 0, 0.65, new Rotation3d(0, 0, 0)); // TODO:
                                                                                                           // figure
    // out
    // transformation
    private final Transform3d AMP_TRANSFORM = new Transform3d(0, 0, -0.5, new Rotation3d(0, 0, 0)); // TODO: figure out
    // transformation

    private final Pose3d BLUE_SPEAKER = field_layout_.getTagPose(7).get().transformBy(SPEAKER_TRANSFORM)
            .transformBy(new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.PI)));
    private final Pose3d RED_SPEAKER = field_layout_.getTagPose(4).get().transformBy(SPEAKER_TRANSFORM);
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
        ACTIVETARGETING,
        IDLE,
        READY,
        TRANSFER
    }

    private ShooterPeriodicIo io_;

    /**
     * Constructor for the example subsystem. The constructor should create all
     * instances of the required hardware as well as the PeriodicIO class defined
     * below. This should not attempt to configure any of the hardware as that
     * should be done in the reset() function.
     */
    public ShooterSubsystem() {
        io_ = new ShooterPeriodicIo();
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
      //  wrist_controller_.setD(ShooterConstants.WRIST_CONTROLLER_D);

        roller_motor_ = new CANSparkMax(ShooterConstants.ROLLER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        roller_motor_.setInverted(true);

        target_pub = NetworkTableInstance.getDefault().getStructTopic("tag_pose", Pose3d.struct).publish();
        rot_pub = NetworkTableInstance.getDefault().getStructTopic("rot_pose", Pose2d.struct).publish();
        reset();

        note_sensor_ = new TimeOfFlight(ShooterConstants.NOTE_SENSOR_ID);
        note_sensor_.setRangingMode(TimeOfFlight.RangingMode.Medium, ShooterConstants.SENSOR_SAMPLE_TIME);
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

    @Override
    /**
     * Inside this function should be logic and code to fully reset your subsystem.
     * This is called during initialization, and should handle I/O configuration and
     * initializing data members.
     */
    public void reset() {
        io_ = new ShooterPeriodicIo();
    }

    @Override
    /**
     * Inside this function, all of the SENSORS should be read into variables stored
     * in the PeriodicIO class defined below. There should be no calls to output to
     * actuators, or any logic within this function.
     */
    public void readPeriodicInputs(double timestamp) {
        io_.current_flywheel_speed_ = top_flywheel_motor_.getEncoder().getVelocity();
        io_.current_wrist_angle_ = wrist_encoder_.getPosition() * (2 * Math.PI) - ShooterConstants.WRIST_ZERO_ANGLE;
        io_.note_sensor_range_ = note_sensor_.getRange();

    }

    @Override
    /**
     * Inside this function, all of the LOGIC should compute updates to output
     * variables in the PeriodicIO class defined below. There should be no calls to
     * read from sensors or write to actuators in this function.
     */
    public void updateLogic(double timestamp) {
        Pose2d robot_pose = PoseEstimator.getInstance().getRobotPose();

        if (io_.target_mode_ == ShootMode.ACTIVETARGETING) {
            io_.target_robot_yaw_ = calculateTargetYaw();
            io_.target_wrist_angle_ = calculateWristAngle(robot_pose, io_.target_, calculateNoteExitVelocity());
            io_.relative_chassis_speed_ = transformChassisVelocity();
        } else if (io_.target_mode_ == ShootMode.IDLE) {
            io_.target_wrist_angle_ = ShooterConstants.WRIST_HOME_ANGLE;
        }

        hasNote();
    }

    @Override
    /**
     * Inside this function actuator OUTPUTS should be updated from data contained
     * in
     * the PeriodicIO class defined below. There should be little to no logic
     * contained within this function, and no sensors should be read.
     */
    public void writePeriodicOutputs(double timestamp) {
        bot_flywheel_motor_.set(io_.target_flywheel_speed_);
        roller_motor_.set(io_.roller_speed_);
        setWristAngle();
        SwerveDrivetrain.getInstance().setTargetRotation(io_.target_robot_yaw_);

    }

    @Override
    /**
     * Inside this function telemetry should be output to smartdashboard. The data
     * should be collected out of the PeriodicIO class instance defined below. There
     * should be no sensor information read in this function nor any outputs made to
     * actuators within this function. Only publish to smartdashboard here.
     */
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

    @Override
    public LogData getLogger() {
        return io_;
    }

    public class ShooterPeriodicIo extends LogData {
        public Pose3d target_ = new Pose3d();
        public double target_flywheel_speed_ = 0;
        public double current_flywheel_speed_ = 0;
        public double target_wrist_angle_ = 0;
        public double current_wrist_angle_ = 0;
        public ShootMode target_mode_ = ShootMode.IDLE;
        public double roller_speed_;
        public boolean has_note_;
        public Rotation2d target_robot_yaw_ = new Rotation2d();
        public double note_travel_time_;
        public Transform3d target_transform_;
        public ChassisSpeeds relative_chassis_speed_;
        public double note_sensor_range_;
    }
}
