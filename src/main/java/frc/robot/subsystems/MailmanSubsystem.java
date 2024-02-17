// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Util;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.MailmanConstants;
import frc.robot.Constants.ShooterConstants;

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
  private MailmanPeriodicIo io_;
  private CANSparkFlex elevator_motor_;
  private RelativeEncoder elevator_encoder_;
  private CANSparkFlex dropper_motor_;
  // TODO: Implement PID Controller on SparkFlex for faster preformance
  // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Position%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
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
    io_ = new MailmanPeriodicIo();
    elevator_motor_ = new CANSparkFlex(Constants.MailmanConstants.ELEVATOR_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    dropper_motor_ = new CANSparkFlex(Constants.MailmanConstants.DROPPER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    elevator_encoder_ = elevator_motor_.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
    heightController = new ProfiledPIDController(Constants.MailmanConstants.HEIGHT_CONTROLLER_P,Constants.MailmanConstants.HEIGHT_CONTROLLER_I,Constants.MailmanConstants.HEIGHT_CONTROLLER_D,Constants.MailmanConstants.HEIGHT_CONTROLLER_CONSTRAINT);
  }

  @Override
  /**
   * Inside this function should be logic and code to fully reset your subsystem.
   * This is called during initialization, and should handle I/O configuration and
   * initializing data members.
   */
  public void reset() {
    io_ = new MailmanPeriodicIo();
  }

  @Override
  /**
   * Inside this function, all of the SENSORS should be read into variables stored
   * in the PeriodicIO class defined below. There should be no calls to output to
   * actuators, or any logic within this function.
   */
  public void readPeriodicInputs(double timestamp) {
    io_.current_height_ = elevator_encoder_.getPosition();

  }

  @Override
  /**
   * Inside this function, all of the LOGIC should compute updates to output
   * variables in the PeriodicIO class defined below. There should be no calls to
   * read from sensors or write to actuators in this function.
   */
  public void updateLogic(double timestamp) {
    io_.controller_output_ = heightController.calculate(io_.current_height_, io_.desired_height_);
  }

  @Override
  /**
   * Inside this function actuator OUTPUTS should be updated from data contained
   * in
   * the PeriodicIO class defined below. There should be little to no logic
   * contained within this function, and no sensors should be read.
   */
  public void writePeriodicOutputs(double timestamp) {
    elevator_motor_.set(io_.controller_output_);
    dropper_motor_.set(io_.roller_speed_);
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
    return io_;
  }

public boolean atHeight(){
  return Util.epislonEquals(io_.current_height_,io_.desired_height_);
}

public void setHeight(HeightTarget target){
  if(target==HeightTarget.AMP){
    io_.desired_height_ = Constants.MailmanConstants.AMP_HEIGHT;
  } else if(target==HeightTarget.TRAP){
    io_.desired_height_ = Constants.MailmanConstants.TRAP_HEIGHT;
  }else{
    io_.desired_height_ = 0.0; //stored elevator height
  }
}

public void setRollerIntake(){
    io_.roller_speed_ = Constants.MailmanConstants.DROPPER_IN_SPEED;
}

public void setRollerOutput(){
    io_.roller_speed_ = Constants.MailmanConstants.DROPPER_OUT_SPEED;
}

public void setRollerStop(){
    io_.roller_speed_ = 0;
}

  public class MailmanPeriodicIo extends LogData {
    public double current_height_ = 0.0;
    public double desired_height_ = 0.0;
    public boolean is_holding_note_ = false;
    public boolean is_allinged_ = false;
    public boolean note_wanted_elsewhere_ = false;
    public double controller_output_ = 0.0;
    public double roller_speed_ = 0.0;
    public boolean has_note_;
    public double note_sensor_range_;
  }
}
