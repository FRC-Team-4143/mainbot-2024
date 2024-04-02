package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.LEDConstants;
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

    private LEDSubsystemPeriodicIo io_;
    private AddressableLED led_ = new AddressableLED(LEDConstants.LED_PORT);
    private AddressableLEDBuffer led_buffer_ = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);

    private BooleanSupplier hasNoteSupplier, passingSupplier, shootingSupplier;
    private BooleanEvent passingEvent, shootingEvent;

    Debouncer pickupNoteDebouncer = new Debouncer(0.5, DebounceType.kFalling);

    private EventLoop led_eventloop_ = new EventLoop();

    LEDSubsystem() {
        io_ = new LEDSubsystemPeriodicIo();

        for (int i = 0; i < led_buffer_.getLength(); i++) {
            led_buffer_.setRGB(i, 0, 255, 0);
        }
        led_.setLength(led_buffer_.getLength());
        led_.setData(led_buffer_);
        led_.start();

        hasNoteSupplier = () -> ShooterSubsystem.getInstance().hasNote() || pickupNoteDebouncer.calculate(PickupSubsystem.getMailmanInstance().hasNote()) || pickupNoteDebouncer.calculate(PickupSubsystem.getShooterInstance().hasNote());
        passingSupplier = () -> ShooterSubsystem.getInstance().getShootTarget() == ShootTarget.PASS;
        shootingSupplier = () -> ShooterSubsystem.getInstance().getShootTarget() == ShootTarget.SPEAKER;

        passingEvent = new BooleanEvent(led_eventloop_, passingSupplier);
        passingEvent.ifHigh(() -> setColorRGBFlash(255, 0, 235, (hasNoteSupplier.getAsBoolean())? io_.led_cycle_state : true)); // Purple

        shootingEvent = new BooleanEvent(led_eventloop_, shootingSupplier);
        shootingEvent.ifHigh(() -> setColorRGBFlash(0, 255, 0, (hasNoteSupplier.getAsBoolean())? io_.led_cycle_state : true)); // Green
    }

    @Override
    public void reset() {
        io_ = new LEDSubsystemPeriodicIo();
        led_buffer_ = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);
        led_.setLength(led_buffer_.getLength());
        led_.setData(led_buffer_);
        led_.start();
    }

    @Override
    public void readPeriodicInputs(double timestamp) {}

    @Override
    public void updateLogic(double timestamp) {
        led_eventloop_.poll();
        ledCycle();

        if(DriverStation.isDisabled()){
            if(DriverStation.getAlliance().isPresent()){
                if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
                    setColorRGBCycle(255, 0, 0, io_.led_cycle_state);
                    setColorRGBCycle(0, 0, 0, !io_.led_cycle_state);

                } else {
                    setColorRGBCycle(0, 0, 255, io_.led_cycle_state);
                    setColorRGBCycle(0, 0, 0, !io_.led_cycle_state);
                }
            } else {
                setColorRGBCycle(255, 0, 0, io_.led_cycle_state);
                setColorRGBCycle(0, 0, 255, !io_.led_cycle_state);
            }
        }
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        led_.setData(led_buffer_);
    }

    @Override
    public void outputTelemetry(double timestamp) {}

    public void setColorRGB(int r, int g, int b) {
        for (int i = 0; i < led_buffer_.getLength(); i++) {
            led_buffer_.setRGB(i, r, g, b);
        }
    }

    public void setColorRGBUpperCycle(int r, int g, int b){
        for (int i = 0; i < led_buffer_.getLength(); i+=2) {
            led_buffer_.setRGB(i, r, g, b);
        } 
    }

    public void setColorRGBLowerCycle(int r, int g, int b){
        for (int i = 1; i < led_buffer_.getLength(); i+=2) {
            led_buffer_.setRGB(i, r, g, b);
        }
    }

    public void setColorRGBCycle(int r, int g, int b, boolean cycle){
        if(cycle){
            setColorRGBUpperCycle(r, g, b);
        } else {
            setColorRGBLowerCycle(r, g, b);
        }
    }

    public void setColorRGBFlash(int r, int g, int b, boolean cycle){
        if(cycle){
            setColorRGB(r, g, b);
        } else {
            setColorRGB(0, 0, 0);
        }
    }

    public void ledCycle(){
        io_.led_cycle_counter--;
        if(io_.led_cycle_counter <= 0){
            io_.led_cycle_state = !io_.led_cycle_state;
            io_.led_cycle_counter = 25;
        }
    }

    public class LEDSubsystemPeriodicIo implements Logged {
        public boolean led_cycle_state = true;
        public int led_cycle_counter = 25;
    }

    @Override
    public Logged getLoggingObject() {
        return io_;
    }
}
