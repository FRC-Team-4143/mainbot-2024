package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.PoseEstimator.PoseEstimatorPeriodicIo;
import frc.robot.subsystems.ShooterSubsystem.ShootTarget;
import monologue.Logged;

public class LEDSubsystem extends Subsystem {

    private static LEDSubsystem LEDSubsystemInstance;

    public static LEDSubsystem getInstance() {
        if (LEDSubsystemInstance == null) {
            LEDSubsystemInstance = new LEDSubsystem();
        }
        return LEDSubsystemInstance;
    }

    private PoseEstimatorPeriodicIo io_;
    private AddressableLED led_ = new AddressableLED(LEDConstants.LED_PORT);
    private AddressableLEDBuffer led_buffer_ = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);

    private BooleanSupplier noNoteSupplier, hasNoteSupplier, targetLockedSupplier, passingSupplier;
    private BooleanEvent noNoteEvent, hasNoteEvent, targetLockedEvent, passingEvent;

    private EventLoop led_eventloop_;

    LEDSubsystem() {
        for (int i = 0; i < led_buffer_.getLength(); i++) {
            led_buffer_.setHSV(i, 0, 0, 0);
        }
        led_.setData(led_buffer_);
        led_.start();

        noNoteEvent = new BooleanEvent(led_eventloop_, noNoteSupplier);
        noNoteEvent.ifHigh(() -> setColorHSV(0, 0, 0)); // Either black or off

        hasNoteEvent = new BooleanEvent(led_eventloop_, hasNoteSupplier);
        hasNoteEvent.ifHigh(() -> setColorHSV(28, 100, 100)); // Orange

        targetLockedEvent = new BooleanEvent(led_eventloop_, targetLockedSupplier);
        targetLockedEvent.ifHigh(() -> setColorHSV(0, 100, 100)); // Red

        passingEvent = new BooleanEvent(led_eventloop_, passingSupplier);
        passingEvent.ifHigh(() -> setColorHSV(60, 100, 100)); // Yellow

        noNoteSupplier = () -> !ShooterSubsystem.getInstance().hasNote() && !PickupSubsystem.getMailmanInstance().hasNote() && !PickupSubsystem.getShooterInstance().hasNote();
        hasNoteSupplier = () -> ShooterSubsystem.getInstance().hasNote() || PickupSubsystem.getMailmanInstance().hasNote() || PickupSubsystem.getShooterInstance().hasNote();
        targetLockedSupplier = () -> ShooterSubsystem.getInstance().isTargetLocked();
        passingSupplier = () -> ShooterSubsystem.getInstance().getShootTarget() == ShootTarget.PASS;

    }

    public void setColorHSV(int h, int s, int v) {
        for (int i = 0; i < led_buffer_.getLength(); i++) {
            led_buffer_.setHSV(i, h, s, v);
        }
    }

    @Override
    public void reset() {
        led_buffer_ = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);
        led_.setData(led_buffer_);
        led_.start();
    }

    @Override
    public void readPeriodicInputs(double timestamp) {}

    @Override
    public void updateLogic(double timestamp) {
        led_eventloop_.poll();
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        led_.setData(led_buffer_);
    }

    @Override
    public void outputTelemetry(double timestamp) {}

    public class LEDSubsystemPeriodicIo implements Logged {}

    @Override
    public Logged getLoggingObject() {
        return io_;
    }
}
