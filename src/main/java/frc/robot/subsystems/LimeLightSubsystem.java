package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ShooterSubsystem.ShootTarget;
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

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry tx = table.getEntry("tx");

    private double limelightLensHeightInches = 20.0; 
    private double limelightMountAngleDegrees = 25.0; 
    private double goalHeightInches = 0.0; 

    public LimeLightSubsystem() {
        io_ = new LimeLightSubsystemIo();
    }

    @Override
    public void reset() {
        
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        io_.limelightTargetX = tx.getDouble(0);
        io_.limelightTargetY = ty.getDouble(0);
    }

    public double calculateDist() {
        double angleToGoalRadians = (limelightMountAngleDegrees + io_.limelightTargetY) * (3.14159 / 180.0);
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    @Override
    public void updateLogic(double timestamp) {
       io_.limelightTargetDist = calculateDist();
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        
    }

    @Override
    public void outputTelemetry(double timestamp) {
        SmartDashboard.putNumber("Dist To Note", io_.limelightTargetDist);
    }

    public class LimeLightSubsystemIo implements Logged {
        @Log.File
        double limelightTargetX = 0;
        @Log.File
        double limelightTargetY = 0;
        @Log.File
        double limelightTargetDist = 0;
    }

    @Override
    public Logged getLoggingObject() {
        return io_;
    }
}
