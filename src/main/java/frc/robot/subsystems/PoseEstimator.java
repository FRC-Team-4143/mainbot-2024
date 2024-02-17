package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.lib.subsystem.Subsystem;

public class PoseEstimator extends Subsystem {

    private static PoseEstimator instance;
    private StructPublisher<Pose2d> pose_pub_;

    public static PoseEstimator getInstance() {
        if (instance == null) {
            instance = new PoseEstimator();
        }
        return instance;
    }

    private PeriodicIo io_;
    private Field2d field;
    private SwerveDrivePoseEstimator odometry;
    private SwerveDrivePoseEstimator vision_filtered_odometry;
    private GenericSubscriber vision_subsciber;

    PoseEstimator() {
        field = new Field2d();
        io_ = new PeriodicIo();
        pose_pub_ = NetworkTableInstance.getDefault().getStructTopic("robot_pose", Pose2d.struct).publish();
        
    }

    @Override
    public void readPeriodicInputs(double timestamp) {

    }
    // Make a subscriber, integate vision measurements wpilib method on the new odometry, getLastChange?
    @Override
    public void updateLogic(double timestamp) {
        var drive = SwerveDrivetrain.getInstance();
        io_.pose_ = odometry.update(drive.getImuYaw(), drive.getModulePositions());
        
        
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
    }

    @Override
    public void outputTelemetry(double timestamp) {
        field.setRobotPose(io_.pose_);
        SmartDashboard.putData("Field", field); // TODO: Smartdashboard currently uses odometry pose
        pose_pub_.set(io_.pose_);
    }

    @Override
    public void reset() {
        odometry = new SwerveDrivePoseEstimator(SwerveDrivetrain.getInstance().kinematics, new Rotation2d(),
                SwerveDrivetrain.getInstance().getModulePositions(), new Pose2d());
        vision_filtered_odometry = new SwerveDrivePoseEstimator(SwerveDrivetrain.getInstance().kinematics, null, null,
                getVisionFilteredPose());
        io_ = new PeriodicIo();
    }

    @Override
    public LogData getLogger() {
        return io_;
    }

    /**
     * 
     * @return
     */
    public Pose2d getRobotPose() {
        return io_.pose_;
    }

    public Pose2d getVisionFilteredPose() {
        return io_.vision_filtered_pose_;
    }

    /**
     * Resets the robot odom and set the robot pose to supplier Pose
     * 
     * @param pose pose to update Robot pose to
     */
    public void setRobotOdometry(Pose2d pose) {
        var drive = SwerveDrivetrain.getInstance();
        odometry.resetPosition(drive.getImuYaw(), drive.getModulePositions(), pose);
        vision_filtered_odometry.resetPosition(drive.getImuYaw(), drive.getModulePositions(), pose);
    }

    class PeriodicIo extends LogData {
        Pose2d pose_ = new Pose2d();
        Pose2d vision_filtered_pose_ = new Pose2d();
    }

}
