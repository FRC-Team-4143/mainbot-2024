// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.Util;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.ShooterConstatnts;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class ShooterSubsystem extends Subsystem {
  // Singleton pattern
  private static ShooterSubsystem ShooterInstance = null;

  public static ShooterSubsystem getInstance() {
    if (ShooterInstance == null) {
      ShooterInstance = new ShooterSubsystem();
    }
    return ShooterInstance;
  }
  
  //initialize motors
  private CANSparkFlex top_flywheel_motor_;
  private CANSparkFlex bot_flywheel_motor_;
  private CANSparkFlex wrist_motor_;
  private CANSparkFlex roller_motor_; //Motor type tbd

  private AprilTagFieldLayout field_layout_;
  
  private final Transform3d SPEAKER_TRANSFORM = new Transform3d(0, 0, 1, new Rotation3d(0, 0, 0)); //TODO: figure out transformation
  private final Transform3d AMP_TRANSFORM = new Transform3d(0, 0, -1, new Rotation3d(0, 0, 0)); //TODO: figure out transformation

  private final Pose3d BLUE_SPEAKER = field_layout_.getTagPose(7).get().transformBy(SPEAKER_TRANSFORM);
  private final Pose3d RED_SPEAKER = field_layout_.getTagPose(4).get().transformBy(SPEAKER_TRANSFORM);
  private final Pose3d BLUE_AMP = field_layout_.getTagPose(6).get().transformBy(AMP_TRANSFORM);
  private final Pose3d RED_AMP = field_layout_.getTagPose(5).get().transformBy(AMP_TRANSFORM);

  private ProfiledPIDController angleControler;

  public enum ShootTarget{
    SPEAKER, 
    AMP
  }

  public enum ShootMode{
    ACTIVETARGETING,
    IDLE,
    READY,
    TRANSFER
  }

  private ShooterPeriodicIo io;

  /**
   * Constructor for the example subsystem. The constructor should create all
   * instances of the required hardware as well as the PeriodicIO class defined
   * below. This should not attempt to configure any of the hardware as that
   * should be done in the reset() function.
   */
  public ShooterSubsystem() {
    io = new ShooterPeriodicIo();

    angleControler = new ProfiledPIDController(ShooterConstatnts.WRIST_CONTROLLER_P, ShooterConstatnts.WRIST_CONTROLLER_I, ShooterConstatnts.WRIST_CONTROLLER_D, ShooterConstatnts.WRIST_CONTROLLER_CONSTRAINT);
    top_flywheel_motor_ = new CANSparkFlex(ShooterConstatnts.TOP_FLYWHEEL_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    bot_flywheel_motor_ = new CANSparkFlex(ShooterConstatnts.BOT_FLYWHEEL_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    wrist_motor_ = new CANSparkFlex(ShooterConstatnts.WRIST_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    roller_motor_ = new CANSparkFlex(ShooterConstatnts.ROLLER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    reset();
  }
  
  //get methods
  public double getShooterAngle(){
    //TODO: return curent angle
    return io.current_wrist_angle_;
  }

  public boolean isTargetLocked(){
    //TODO: is curently aiming at target
    return Util.epislonEquals(io.current_wrist_angle_, io.target_wrist_angle_, ShooterConstatnts.WRIST_TOLERANCE) &&
    Util.epislonEquals(io.current_flywheel_speed_, io.target_flywheel_speed_, ShooterConstatnts.FLYWHEEL_TOLERANCE);
  }

  public boolean hasNote(){
    //TODO: not is in holding postion
    return io.has_note_;
  }


  //set methods
  public void setAngle(double goal){
    angleControler.setGoal(goal);
  }

  public void setTarget(ShootTarget target){
    var alliance = DriverStation.getAlliance();
    if(DriverStation.Alliance.Red == alliance.get()){
      if(target == ShootTarget.SPEAKER){
        io.target_ = RED_SPEAKER;
      } else {
        io.target_ = RED_AMP;
      }
    } else {
      if(target == ShootTarget.SPEAKER){
        io.target_ = BLUE_SPEAKER;
      } else {
        io.target_ = BLUE_AMP;
      }
    }
  }

  public void setFeedSpeed(){

  } 

  public void setFlyWheelSpeed(){

  }

  private double calcuateAngle(Pose3d robot_pose, Pose3d target_pose, double velocity ){
    return 0.0;
  }
    
  
  
  @Override
  /**
   * Inside this function should be logic and code to fully reset your subsystem.
   * This is called during initialization, and should handle I/O configuration and
   * initializing data members.
   */
  public void reset() {
    io = new ShooterPeriodicIo();
  }

  @Override
  /**
   * Inside this function, all of the SENSORS should be read into variables stored
   * in the PeriodicIO class defined below. There should be no calls to output to
   * actuators, or any logic within this function.
   */
  public void readPeriodicInputs(double timestamp) {

  }

  @Override
  /**
   * Inside this function, all of the LOGIC should compute updates to output
   * variables in the PeriodicIO class defined below. There should be no calls to
   * read from sensors or write to actuators in this function.
   */
  public void updateLogic(double timestamp) {

  }

  @Override
  /**
   * Inside this function actuator OUTPUTS should be updated from data contained in
   * the PeriodicIO class defined below. There should be little to no logic
   * contained within this function, and no sensors should be read.
   */
  public void writePeriodicOutputs(double timestamp) {

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

  public class ShooterPeriodicIo extends LogData {
    public Pose3d target_;
    public double target_flywheel_speed_;
    public double current_flywheel_speed_;
    public double target_wrist_angle_;
    public double current_wrist_angle_;
    public ShootMode target_mode_ = ShootMode.IDLE;
    public double roller_speed_;
    public boolean has_note_;
  }
}
