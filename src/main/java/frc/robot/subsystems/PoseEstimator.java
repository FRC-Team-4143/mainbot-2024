package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.ProtobufSubscriber;
import edu.wpi.first.networktables.TimestampedObject;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.subsystem.Subsystem;

public class PoseEstimator extends Subsystem {

    private static PoseEstimator poseEstimatorInstance;

    public static PoseEstimator getInstance() {
        if (poseEstimatorInstance == null) {
            poseEstimatorInstance = new PoseEstimator();
        }
        return poseEstimatorInstance;
    }

    private PoseEstimatorPeriodicIoAutoLogged io_;
    private Field2d field_;
    private SwerveDrivePoseEstimator odometry_;
    private SwerveDrivePoseEstimator vision_filtered_odometry_;
    private ProtobufSubscriber<Pose2d> vision_subsciber_;
    private BooleanPublisher supression_publisher_;
    private BooleanSubscriber vision_ready_subscriber_;
    private ProtobufPublisher<Pose2d> odom_publisher_;
    private ProtobufPublisher<Pose2d> pose_publisher_;
   
    private Transform2d vision_box_transform_ = new Transform2d(new Translation2d(0.33,-0.457), Rotation2d.fromDegrees(180));//30+90));
   
    int update_counter_ = 10;

    PoseEstimator() {
        io_ = new PoseEstimatorPeriodicIoAutoLogged();
        field_ = new Field2d();

        NetworkTableInstance nti = NetworkTableInstance.getDefault();
        var field_pose_topic = nti.getProtobufTopic("vision/pose", Pose2d.proto);
        var robot_odom_topic = nti.getProtobufTopic("vision/odom", Pose2d.proto);
        var robot_pose_topic = nti.getProtobufTopic("robotpose", Pose2d.proto);
        var supress_odom_topic = nti.getBooleanTopic("vision/supress_odom");
        var vision_ready_topic = nti.getBooleanTopic("vision/ready");

        vision_subsciber_ = field_pose_topic.subscribe(new Pose2d());
        vision_ready_subscriber_ = vision_ready_topic.subscribe(false);
        odom_publisher_ = robot_odom_topic.publish();
        pose_publisher_ = robot_pose_topic.publish();
        supression_publisher_ = supress_odom_topic.publish(); 
    }

    @Override
    public void reset() {
        io_ = new PoseEstimatorPeriodicIoAutoLogged();
        odometry_ = new SwerveDrivePoseEstimator(SwerveDrivetrain.getInstance().kinematics, new Rotation2d(),
                SwerveDrivetrain.getInstance().getModulePositions(), new Pose2d());
        vision_filtered_odometry_ = new SwerveDrivePoseEstimator(SwerveDrivetrain.getInstance().kinematics,
                new Rotation2d(), SwerveDrivetrain.getInstance().getModulePositions(),
                new Pose2d());
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        TimestampedObject<Pose2d> result = vision_subsciber_.getAtomic();
    
        if (result.timestamp > io_.last_vision_timestamp_) {
            vision_filtered_odometry_.addVisionMeasurement(result.value.transformBy(vision_box_transform_), timestamp - 0.2);
            io_.last_vision_timestamp_ = result.timestamp;
        }
        SmartDashboard.putNumber("Vision Timestamp", result.timestamp);
        SmartDashboard.putNumber("Subsystem Timestamp", timestamp);
    }

    // Make a subscriber, integate vision measurements wpilib method on the new
    // odometry, getLastChange?
    @Override
    public void updateLogic(double timestamp) {
        var drive = SwerveDrivetrain.getInstance();
        io_.pose_ = odometry_.update(drive.getImuYaw(), drive.getModulePositions());
        io_.vision_filtered_pose_ = vision_filtered_odometry_.update(drive.getImuYaw(), drive.getModulePositions());
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        update_counter_--;
        if(update_counter_ == 0){
            odom_publisher_.set(io_.pose_);
            update_counter_ = 10;
        }
    }

    @Override
    public void outputTelemetry(double timestamp) {
        field_.setRobotPose(io_.vision_filtered_pose_);
        SmartDashboard.putData("Field", field_);
        pose_publisher_.set(io_.vision_filtered_pose_);
    }

    public Pose2d getRobotPose() {
        return io_.vision_filtered_pose_;
    }

    public SwerveDrivePoseEstimator getOdometryPose() {
        return vision_filtered_odometry_;
    }

    /**
     * Resets the robot odom and set the robot pose to supplier Pose
     * 
     * @param pose pose to update Robot pose to
     */
    public void setRobotOdometry(Pose2d pose) {
        var drive = SwerveDrivetrain.getInstance();
        SwerveDrivetrain.getInstance().seedFieldRelative(pose.getRotation());
        odometry_.resetPosition(drive.getImuYaw(), drive.getModulePositions(), pose);
        vision_filtered_odometry_.resetPosition(drive.getImuYaw(), drive.getModulePositions(), pose);
    }

    @AutoLog
    public static class PoseEstimatorPeriodicIo extends LogData {
        Pose2d pose_ = new Pose2d();
        Pose2d vision_filtered_pose_ = new Pose2d();
        double last_vision_timestamp_ = 0.0;
        boolean vision_supress_odom_ = true;
    }

    @Override
    public LoggableInputs getLogger() {
        return io_;
    }
}
