package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.lib.subsystem.Subsystem;

public class PoseEstimator extends Subsystem {

    private static PoseEstimator instance;

    public static PoseEstimator getInstance() {
        if (instance == null) {
            instance = new PoseEstimator();
        }
        return instance;
    }

    private PeriodicIo io = new PeriodicIo();
    private final Field2d field = new Field2d();
    private SwerveDrivePoseEstimator odometry;

    PoseEstimator() {
        odometry = new SwerveDrivePoseEstimator(SwerveDrivetrain.getInstance().kinematics, new Rotation2d(),
                SwerveDrivetrain.getInstance().getModulePositions(), new Pose2d());
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
    
    }

    @Override
    public void updateLogic(double timestamp) {
        var drive = SwerveDrivetrain.getInstance();
        io.pose = odometry.update(drive.getImuYaw(), drive.getModulePositions());
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
    }

    @Override
    public void outputTelemetry(double timestamp) {
        field.setRobotPose(io.pose);
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void reset() {
        
    }

    @Override
    public LogData getLogger() {
        return io;
    }

    /**
     * 
     * @return
     */
    public Pose2d getRobotPose(){
        return io.pose;
    }

    /**
     * Resets the robot odom and set the robot pose to supplier Pose
     * @param pose pose to update Robot pose to
     */
    public void setRobotOdometry(Pose2d pose){
        var drive = SwerveDrivetrain.getInstance();
        odometry.resetPosition(drive.getImuYaw(), drive.getModulePositions(), pose);
    }

    class PeriodicIo extends LogData {
        Pose2d pose = new Pose2d();
    }

}
