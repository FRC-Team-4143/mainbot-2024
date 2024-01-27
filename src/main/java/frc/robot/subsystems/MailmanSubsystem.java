// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.lib.Util;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants;

public class MailmanSubsystem extends Subsystem {

  // Singleton pattern
  private static MailmanSubsystem exampleInstance = null;

  public static MailmanSubsystem getInstance() {
    if (exampleInstance == null) {
      exampleInstance = new MailmanSubsystem();
    }
    return exampleInstance;
  }

  /**
   * 
   */
  private PeriodicIo io;
  private CANSparkFlex elevatorMotor;
  private CANSparkFlex dropperMotor;
  private RelativeEncoder elevatorEncoder;
  private ProfiledPIDController heightController;
  public enum HeightTarget{
    AMP,
    TRAP,
    HOME
  }

  /**
   * Constructor for the example subsystem. The constructor should create all
   * instances of the required hardware as well as the PeriodicIO class defined
   * below. This should not attempt to configure any of the hardware as that
   * should be done in the reset() function.
   */
  private MailmanSubsystem() {
    io = new PeriodicIo();
    elevatorMotor = new CANSparkFlex(Constants.MailmanConstants.elevator_motor_id,
        CANSparkLowLevel.MotorType.kBrushless);
    dropperMotor = new CANSparkFlex(Constants.MailmanConstants.dropper_motor_id, CANSparkLowLevel.MotorType.kBrushless);
    elevatorEncoder = elevatorMotor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
    heightController = new ProfiledPIDController(Constants.MailmanConstants.k_height_controllerP,Constants.MailmanConstants.k_height_controllerI,Constants.MailmanConstants.k_height_controllerD,Constants.MailmanConstants.height_controller_constraint);
  }

  @Override
  /**
   * Inside this function should be logic and code to fully reset your subsystem.
   * This is called during initialization, and should handle I/O configuration and
   * initializing data members.
   */
  public void reset() {
    io = new PeriodicIo();
  }

  @Override
  /**
   * Inside this function, all of the SENSORS should be read into variables stored
   * in the PeriodicIO class defined below. There should be no calls to output to
   * actuators, or any logic within this function.
   */
  public void readPeriodicInputs(double timestamp) {
    io.current_height = elevatorEncoder.getPosition();

  }

  @Override
  /**
   * Inside this function, all of the LOGIC should compute updates to output
   * variables in the PeriodicIO class defined below. There should be no calls to
   * read from sensors or write to actuators in this function.
   */
  public void updateLogic(double timestamp) {
    io.controller_output = heightController.calculate(io.current_height, io.wanted_height);
  }

  @Override
  /**
   * Inside this function actuator OUTPUTS should be updated from data contained
   * in
   * the PeriodicIO class defined below. There should be little to no logic
   * contained within this function, and no sensors should be read.
   */
  public void writePeriodicOutputs(double timestamp) {
    elevatorMotor.set(io.controller_output);
    dropperMotor.set(io.roller_speed);
  }

  @Override
  /**
   * Inside this function telemetry should be output to smartdashboard. The data
   * should be collected out of the PeriodicIO class instance defined below. There
   * should be no sensor information read in this function nor any outputs made to
   * actuators within this function. Only publish to smartdashboard here.
   */
  public void outputTelemetry(double timestamp) {

  }

  @Override
  public LogData getLogger() {
    return io;
  }

public boolean atHeight(){
  return Util.epislonEquals(io.current_height,io.wanted_height);
}

public void setHeight(HeightTarget desiredHeight){
  if(desiredHeight==HeightTarget.AMP){
    io.wanted_height = Constants.MailmanConstants.amp_height;
  } else if(desiredHeight==HeightTarget.TRAP){
    io.wanted_height = Constants.MailmanConstants.trap_height;
  }else{
    io.wanted_height = 0.0; //stored elevator height
  }
}

public void setRollerIntake(){
    io.roller_speed = Constants.MailmanConstants.intake_speed;
}

public void setRollerOutput(){
    io.roller_speed = Constants.MailmanConstants.output_speed;
}

public void setRollerStop(){
    io.roller_speed = 0;
}

  public class PeriodicIo extends LogData {
    public double current_height = 0.0;
    public double wanted_height = 0.0;
    public boolean is_holding_note = false;
    public boolean is_allinged = false;
    public boolean note_wanted_elsewhere = false;
    public double controller_output = 0.0;
    public double roller_speed = 0.0;
  }
}
