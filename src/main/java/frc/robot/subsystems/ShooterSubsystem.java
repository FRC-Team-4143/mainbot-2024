// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.Util;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

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
  private CANSparkMax roller_motor_; //Motor type tbd

  private AprilTagFieldLayout field_layout_= AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  
  private final Transform3d SPEAKER_TRANSFORM = new Transform3d(0, 0, 1, new Rotation3d(0, 0, 0)); //TODO: figure out transformation
  private final Transform3d AMP_TRANSFORM = new Transform3d(0, 0, -1, new Rotation3d(0, 0, 0)); //TODO: figure out transformation

  private final Pose3d BLUE_SPEAKER = field_layout_.getTagPose(7).get().transformBy(SPEAKER_TRANSFORM);
  private final Pose3d RED_SPEAKER = field_layout_.getTagPose(4).get().transformBy(SPEAKER_TRANSFORM);
  private final Pose3d BLUE_AMP = field_layout_.getTagPose(6).get().transformBy(AMP_TRANSFORM);
  private final Pose3d RED_AMP = field_layout_.getTagPose(5).get().transformBy(AMP_TRANSFORM);

  private ProfiledPIDController angle_controller_;

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

  private ShooterPeriodicIo io_;

  /**
   * Constructor for the example subsystem. The constructor should create all
   * instances of the required hardware as well as the PeriodicIO class defined
   * below. This should not attempt to configure any of the hardware as that
   * should be done in the reset() function.
   */
  public ShooterSubsystem() {
    io_ = new ShooterPeriodicIo();
    angle_controller_ = new ProfiledPIDController(ShooterConstants.ANGLE_CONTROLLER_P, ShooterConstants.ANGLE_CONTROLLER_I, ShooterConstants.ANGLE_CONTROLLER_D, ShooterConstants.ANGLE_CONTROLLER_CONSTRAINT);
    top_flywheel_motor_ = new CANSparkFlex(ShooterConstants.TOP_FLYWHEEL_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    bot_flywheel_motor_ = new CANSparkFlex(ShooterConstants.BOT_FLYWHEEL_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    top_flywheel_motor_.follow(bot_flywheel_motor_, true);
    wrist_motor_ = new CANSparkFlex(ShooterConstants.WRIST_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    roller_motor_ = new CANSparkMax(ShooterConstants.ROLLER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    reset();
  }
  
  //get methods
  public double getShooterAngle(){
    //TODO: return curent angle
    return io_.current_wrist_angle_;
  }

  public boolean isTargetLocked(){
    //TODO: is curently aiming at target
    return Util.epislonEquals(io_.current_wrist_angle_, io_.target_wrist_angle_, ShooterConstants.ANGLE_TOLERANCE) &&
    Util.epislonEquals(io_.current_flywheel_speed_, io_.target_flywheel_speed_, ShooterConstants.FLYWHEEL_TOLERANCE);
  }

  public boolean hasNote(){
    //TODO: not is in holding postion
    return io_.has_note_;
  }


  //set methods
  public void setAngle(double goal){
    angle_controller_.setGoal(goal);
  }

  public void setTarget(ShootTarget target){
    var alliance = DriverStation.getAlliance();
    if(DriverStation.Alliance.Red == alliance.get()){
      if(target == ShootTarget.SPEAKER){
        io_.target_ = RED_SPEAKER;
      } else {
        io_.target_ = RED_AMP;
      }
    } else {
      if(target == ShootTarget.SPEAKER){
        io_.target_ = BLUE_SPEAKER;
      } else {
        io_.target_ = BLUE_AMP;
      }
    }
  }

  public void setRollerFeed(){
    io_.roller_speed_ = ShooterConstants.ROLLER_SPEED;
  } 

  public void setRollerReverse(){
    io_.roller_speed_ = -ShooterConstants.ROLLER_SPEED;
  } 

  public void rollerStop(){
    io_.roller_speed_ = 0;
  }
  public void setWristSpeed(double wristSpeed){
    io_.wrist_speed_ = wristSpeed;
  } 
  public void wristStop(){
    io_.wrist_speed_ = 0;
  }

  // TODO: This method should either be rewritten or only used for manual overrides
  // THIS IS ONLY FOR PROTOTYPE TESTING!!!!
  public void setFlyWheelSpeed(double speed){
    io_.target_flywheel_speed_ =  speed;
  }

  public void flyWheelStop(){
    io_.target_flywheel_speed_ =  0;
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
    io_ = new ShooterPeriodicIo();
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
    bot_flywheel_motor_.set(io_.target_flywheel_speed_);
    roller_motor_.set(io_.roller_speed_);
    wrist_motor_.set(io_.wrist_speed_);
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

  public class ShooterPeriodicIo extends LogData {
    public Pose3d target_;
    public double target_flywheel_speed_;
    public double current_flywheel_speed_;
    public double target_wrist_angle_;
    public double current_wrist_angle_;
    public ShootMode target_mode_ = ShootMode.IDLE;
    public double roller_speed_;
    public boolean has_note_;
    public double wrist_speed_;
  }
}
