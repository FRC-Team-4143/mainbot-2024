package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.LimelightConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class LimeLightSubsystem extends Subsystem {

    private static LimeLightSubsystem LimeLightSubsystemInstance;

    public static LimeLightSubsystem getInstance() {
        if (LimeLightSubsystemInstance == null) {
            LimeLightSubsystemInstance = new LimeLightSubsystem();
        }
        return LimeLightSubsystemInstance;
    }

    private LimeLightSubsystemIo io_;

    private NetworkTable table;
    private NetworkTableEntry table_entry_tx, table_entry_ty, table_entry_ta, table_entry_tv;
    private ProtobufPublisher<Pose2d> note_pose_pub_;

    public LimeLightSubsystem() {
        io_ = new LimeLightSubsystemIo();
        table = NetworkTableInstance.getDefault().getTable("limelight");
        table_entry_ty = table.getEntry("ty");
        table_entry_tx = table.getEntry("tx");
        table_entry_ta = table.getEntry("ta");
        table_entry_tv = table.getEntry("tv");

        note_pose_pub_ = NetworkTableInstance.getDefault().getProtobufTopic("note pose", Pose2d.proto).publish();
    }

    @Override
    public void reset() {

    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        io_.limelight_target_x_ = -Math.toRadians(table_entry_tx.getDouble(0));
        io_.limelight_target_y_ = -Math.toRadians(table_entry_ty.getDouble(0));
        io_.target_accuracy_ = table_entry_ta.getDouble(0.0);
        io_.target_valid_ = table_entry_tv.getInteger(0);
    }

    public double calculateDist(double y) {
        double angleToGoal = LimelightConstants.MOUNT_ANGLE + y;
        return (LimelightConstants.NOTE_HEIGHT_METERS - LimelightConstants.LENS_HEIGHT_OFF_GROUND_METERS)
                / Math.tan(angleToGoal);
    }

    private Pose2d caluclateNotePose(Pose2d pose) {
        double dist = calculateDist(io_.limelight_target_y_);
        Pose2d result_pose = pose.plus(LimelightConstants.LIMELIGHT_OFFSET);
        return result_pose.plus(new Transform2d(Math.cos(io_.limelight_target_x_) * dist,
                Math.sin(io_.limelight_target_x_) * dist, getNoteRotation(pose)));
    }

    private Rotation2d getNoteRotation(Pose2d pose) {
        return pose.getRotation().rotateBy(Rotation2d.fromRadians(io_.limelight_target_x_));
    }

    public Pose2d getRobotNotePose() {
        return io_.robot_note_pose_;
    }

    @Override
    public void updateLogic(double timestamp) {
        io_.target_distance_ = calculateDist(io_.limelight_target_y_);
        if (io_.target_valid_ == 1) {
            io_.robot_note_pose_ = caluclateNotePose(PoseEstimator.getInstance().getOdomPose());
            io_.field_note_pose_ = caluclateNotePose(PoseEstimator.getInstance().getRobotPose());
        }
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {

    }

    @Override
    public void outputTelemetry(double timestamp) {
        SmartDashboard.putNumber("Dist To Note", io_.target_distance_);
        note_pose_pub_.set(io_.robot_note_pose_);

    }

    public class LimeLightSubsystemIo implements Logged {
        @Log.File
        double limelight_target_x_ = 0.0;
        @Log.File
        double limelight_target_y_ = 0.0;
        @Log.File
        double target_accuracy_ = 0.0;
        @Log.File
        double target_distance_ = 0.0;
        @Log.File
        long target_valid_ = 0;
        @Log.File
        Pose2d robot_note_pose_ = new Pose2d();
        @Log.File
        Pose2d field_note_pose_ = new Pose2d();
    }

    @Override
    public Logged getLoggingObject() {
        return io_;
    }
}
