package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private NetworkTable table_;
    private NetworkTableEntry table_entry_tx_, table_entry_ty_, table_entry_ta_, table_entry_tv_;
    private ProtobufPublisher<Pose2d> note_pose_pub_;

    private Debouncer rising_debouncer_ = new Debouncer(LimelightConstants.NOTE_DETECT_RISING,
            Debouncer.DebounceType.kRising);
    private Debouncer falling_debouncer_ = new Debouncer(LimelightConstants.NOTE_DETECT_FALLING,
            Debouncer.DebounceType.kFalling);

    public LimeLightSubsystem() {
        io_ = new LimeLightSubsystemIo();
        table_ = NetworkTableInstance.getDefault().getTable("limelight");
        table_entry_ty_ = table_.getEntry("ty");
        table_entry_tx_ = table_.getEntry("tx");
        table_entry_ta_ = table_.getEntry("ta");
        table_entry_tv_ = table_.getEntry("tv");

        note_pose_pub_ = NetworkTableInstance.getDefault().getProtobufTopic("note pose", Pose2d.proto).publish();
    }

    @Override
    public void reset() {

    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        io_.limelight_target_x_ = -Math.toRadians(table_entry_tx_.getDouble(0));
        io_.limelight_target_y_ = -Math.toRadians(table_entry_ty_.getDouble(0));
        io_.target_accuracy_ = table_entry_ta_.getDouble(0.0);
        io_.target_valid_ = table_entry_tv_.getInteger(0);
    }

    public double calculateDist(double y) {
        double angleToGoal = LimelightConstants.MOUNT_ANGLE + y;
        return (LimelightConstants.NOTE_HEIGHT_METERS - LimelightConstants.LENS_HEIGHT_OFF_GROUND_METERS)
                / Math.tan(angleToGoal);
    }

    private Pose2d caluclateNotePose(Pose2d pose, double dist, double x) {
        Pose2d result_pose = pose.plus(LimelightConstants.LIMELIGHT_OFFSET);
        return result_pose.plus(new Transform2d(Math.cos(x) * dist,
                Math.sin(x) * dist, Rotation2d.fromRadians(x)));
    }

    public Pose2d getNotePoseOdomRef() {
        return io_.note_pose_odom_ref_;
    }

    public Command updateNoteRangeFlag() {
        return Commands.runOnce(() -> io_.near_note_flag_ = true);
    }

    public void resetNoteRangeFlag() {
        io_.near_note_flag_ = false;
    }

    public boolean isNoteAvaibale() {
        return io_.near_note_flag_ && io_.can_see_note_latch_;
    }

    @Override
    public void updateLogic(double timestamp) {
        io_.can_see_note_ = rising_debouncer_.calculate(io_.target_valid_ == 1);
        io_.can_see_note_latch_ = falling_debouncer_.calculate(io_.can_see_note_);
        if (io_.can_see_note_) {
            io_.target_distance_ = calculateDist(io_.limelight_target_y_);
            io_.note_pose_odom_ref_ = caluclateNotePose(PoseEstimator.getInstance().getOdomPose(), io_.target_distance_,
                    io_.limelight_target_x_);
            io_.note_pose_field_ref_ = caluclateNotePose(PoseEstimator.getInstance().getFieldPose(),
                    io_.target_distance_, io_.limelight_target_x_);
        } else if (io_.can_see_note_latch_) {

        } else {
            io_.note_pose_odom_ref_ = PoseEstimator.getInstance().getOdomPose();
            io_.note_pose_field_ref_ = PoseEstimator.getInstance().getFieldPose();
        }
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {

    }

    @Override
    public void outputTelemetry(double timestamp) {
        SmartDashboard.putNumber("Dist To Note", io_.target_distance_);
        SmartDashboard.putBoolean("Can see note", io_.can_see_note_);
        SmartDashboard.putBoolean("Can see note latch", io_.can_see_note_latch_);
        SmartDashboard.putBoolean("Near note", io_.near_note_flag_);
        note_pose_pub_.set(io_.note_pose_odom_ref_);
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
        Pose2d note_pose_odom_ref_ = new Pose2d();
        @Log.File
        Pose2d note_pose_field_ref_ = new Pose2d();
        @Log.File
        boolean can_see_note_ = false;
        @Log.File
        boolean can_see_note_latch_ = false;
        @Log.File
        boolean near_note_flag_ = false;
    }

    @Override
    public Logged getLoggingObject() {
        return io_;
    }
}
