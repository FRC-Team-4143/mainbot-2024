package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.ProtobufSubscriber;
import edu.wpi.first.networktables.TimestampedObject;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.opencv.dnn.Net;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Util;
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
    private ProtobufPublisher<Pose2d> odom_publisher_;
    private ProtobufPublisher<Pose2d> pose_publisher_;
    private ProtobufPublisher<Pose2d> nvidia0_publisher;
    private ProtobufPublisher<Pose2d> nvidia1_publisher;

    private NetworkTable nvidia0_;
    private NetworkTable nvidia1_;
    private Transform2d nvidia0_transform_ = new Transform2d(new Translation2d(-0.35,0.35), Rotation2d.fromDegrees(0));//30+90));
    private Transform2d nvidia1_transform_ = new Transform2d(new Translation2d(-0.35,0.35), Rotation2d.fromDegrees(0));//-30+90));
    private Transform2d vision_box_transform_ = new Transform2d(new Translation2d(0.33,-0.457), Rotation2d.fromDegrees(180));//30+90));
    private Pose2d nvidia0_result_pose_;
    private Pose2d nvidia1_result_pose_;

    int counter = 10;

    PoseEstimator() {
        io_ = new PoseEstimatorPeriodicIoAutoLogged();
        field_ = new Field2d();

        NetworkTableInstance nti = NetworkTableInstance.getDefault();
        var field_pose_topic = nti.getProtobufTopic("vision/pose", Pose2d.proto);
        var robot_odom_topic = nti.getProtobufTopic("vision/odom", Pose2d.proto);
        var robot_pose_topic = nti.getProtobufTopic("robotpose", Pose2d.proto);

        vision_subsciber_ = field_pose_topic.subscribe(new Pose2d());
        odom_publisher_ = robot_odom_topic.publish();
        pose_publisher_ = robot_pose_topic.publish();

        nvidia0_= NetworkTableInstance.getDefault().getTable("WarVision");
        nvidia1_= NetworkTableInstance.getDefault().getTable("WarVision1");
        var nvidia0_topic = nti.getProtobufTopic("vision/nvidia0", Pose2d.proto);
        var nvidia1_topic = nti.getProtobufTopic("vision/nvidia1", Pose2d.proto);
        nvidia0_publisher = nvidia0_topic.publish();
        nvidia1_publisher = nvidia1_topic.publish(); 
        }

    @Override
    public void reset() {
        odometry_ = new SwerveDrivePoseEstimator(SwerveDrivetrain.getInstance().kinematics, new Rotation2d(),
                SwerveDrivetrain.getInstance().getModulePositions(), new Pose2d());
        vision_filtered_odometry_ = new SwerveDrivePoseEstimator(SwerveDrivetrain.getInstance().kinematics,
                new Rotation2d(), SwerveDrivetrain.getInstance().getModulePositions(),
                new Pose2d());
        // vision_filtered_odometry.setVisionMeasurementStdDevs()
        io_ = new PoseEstimatorPeriodicIoAutoLogged();
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        TimestampedObject<Pose2d> result = vision_subsciber_.getAtomic();
        // Translation2d nvidia0_result_cord = new Translation2d(nvidia0_.getEntry("botposeX").getDouble(0), nvidia0_.getEntry("botposeY").getDouble(0));
        // Translation2d nvidia1_result_cord = new Translation2d(nvidia1_.getEntry("botposeX").getDouble(0), nvidia1_.getEntry("botposeY").getDouble(0));
        // Rotation2d nvidia0_result_rot = new Rotation2d(nvidia0_.getEntry("EulerAngleZ").getDouble(0)+Math.toDegrees(30+90));
        // Rotation2d nvidia1_result_rot = new Rotation2d(nvidia1_.getEntry("EulerAngleZ").getDouble(0)+Math.toDegrees(-30+90));
        // double nvidia0_num_dects = nvidia0_.getEntry("numdetections").getDouble(0);
        // double nvidia1_num_dects = nvidia1_.getEntry("numdetections").getDouble(0);

        // Pose2d nvidia0_result = new Pose2d(nvidia0_result_cord, nvidia0_result_rot);
        // Pose2d nvidia1_result = new Pose2d(nvidia1_result_cord, nvidia1_result_rot);

        // nvidia0_result_pose_ = nvidia0_result.transformBy(nvidia0_transform_);
        // nvidia1_result_pose_ = nvidia1_result.transformBy(nvidia1_transform_);


        // if (nvidia0_num_dects > 1){
        //     vision_filtered_odometry_.addVisionMeasurement(nvidia0_result_pose_, 
        //     timestamp - 0.03, 
        //     new MatBuilder<>(Nat.N3(), Nat.N1()).fill(1/nvidia0_num_dects, 1/nvidia0_num_dects, 0.5/nvidia0_num_dects));
        // }

        // if (nvidia1_num_dects > 1){
        //     vision_filtered_odometry_.addVisionMeasurement(nvidia1_result_pose_, 
        //      timestamp - 0.03, 
        //     new MatBuilder<>(Nat.N3(), Nat.N1()).fill(1/nvidia0_num_dects, 1/nvidia0_num_dects, 0.5/nvidia0_num_dects));
        // }

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
        counter--;
        if(counter == 0){
            odom_publisher_.set(io_.pose_);
            counter = 10;
        }
        // nvidia0_publisher.set(nvidia0_result_pose_);
        // nvidia1_publisher.set(nvidia1_result_pose_);
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
        odometry_.resetPosition(drive.getImuYaw(), drive.getModulePositions(), pose);
        vision_filtered_odometry_.resetPosition(drive.getImuYaw(), drive.getModulePositions(), pose);
    }

    @AutoLog
    public static class PoseEstimatorPeriodicIo extends LogData {
        Pose2d pose_ = new Pose2d();
        Pose2d vision_filtered_pose_ = new Pose2d();
        double last_vision_timestamp_ = 0.0;
    }

    @Override
    public LoggableInputs getLogger() {
        return io_;
    }
}
