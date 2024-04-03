package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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
    private NetworkTableEntry table_entry_ty;
    private NetworkTableEntry table_entry_tx;

    public LimeLightSubsystem() {
        io_ = new LimeLightSubsystemIo();
        table = NetworkTableInstance.getDefault().getTable("limelight");
        table_entry_ty = table.getEntry("ty");
        table_entry_ty = table.getEntry("tx");
    }

    @Override
    public void reset() {
        
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        io_.limelight_target_x_ = table_entry_tx.getDouble(0);
        io_.limelight_target_y_ = table_entry_ty.getDouble(0);
    }

    public double calculateDist(double y) {
        double angleToGoalRadians = Math.toRadians(LimelightConstants.MOUNT_ANGLE_DEGREES + y);
        return (LimelightConstants.goalHeightInches - LimelightConstants.LENS_HEIGHT_METERS) / Math.tan(angleToGoalRadians);
    }

    @Override
    public void updateLogic(double timestamp) {
       io_.target_distance_ = calculateDist(io_.limelight_target_y_);
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        
    }

    @Override
    public void outputTelemetry(double timestamp) {
        SmartDashboard.putNumber("Dist To Note", io_.target_distance_);
    }

    public class LimeLightSubsystemIo implements Logged {
        @Log.File
        double limelight_target_x_ = 0.0;
        @Log.File
        double limelight_target_y_ = 0.0;
        @Log.File
        double target_distance_ = 0.0;
    }

    @Override
    public Logged getLoggingObject() {
        return io_;
    }
}
