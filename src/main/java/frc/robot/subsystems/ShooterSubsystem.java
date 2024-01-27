// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.Util;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.ShooterConstatnts;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.apriltag.AprilTag;
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
  private CANSparkFlex flyWheelTop;
  private CANSparkFlex flyWheelBottom;
  private CANSparkFlex pivotMotor;
  private CANSparkFlex rollerMotor; //Motor type tbd

  private AprilTagFieldLayout fieldLayout;
  
  private final Rotation3d zeroRotation = new Rotation3d(0, 0, 0);
  private final Transform3d speakerTransform = new Transform3d(0, 0, 1, zeroRotation); //TODO: figure out transformation
  private final Transform3d ampTransform = new Transform3d(0, 0, -1, zeroRotation); //TODO: figure out transformation

  private final Pose3d blueSpeaker = fieldLayout.getTagPose(7).get().transformBy(speakerTransform);
  private final Pose3d redSpeaker = fieldLayout.getTagPose(4).get().transformBy(speakerTransform);
  private final Pose3d blueAmp = fieldLayout.getTagPose(6).get().transformBy(ampTransform);
  private final Pose3d redAmp = fieldLayout.getTagPose(5).get().transformBy(ampTransform);

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

    angleControler = new ProfiledPIDController(ShooterConstatnts.kAngleControlerP, ShooterConstatnts.kAngleControlerI, ShooterConstatnts.kAngleControlerD, ShooterConstatnts.angleControlerConstraint);
    flyWheelTop = new CANSparkFlex(ShooterConstatnts.topFlyWheelID, CANSparkLowLevel.MotorType.kBrushless);
    flyWheelBottom = new CANSparkFlex(ShooterConstatnts.bottomFlyWheelID, CANSparkLowLevel.MotorType.kBrushless);
    pivotMotor = new CANSparkFlex(ShooterConstatnts.shooterPivotID, CANSparkLowLevel.MotorType.kBrushless);
    rollerMotor = new CANSparkFlex(ShooterConstatnts.rollerID, CANSparkLowLevel.MotorType.kBrushless);
    reset();
  }
  
  //get methods
  public double getShooterAngle(){
    //TODO: return curent angle
    return io.current_wrist_angle;
  }

  public boolean isTargetLocked(){
    //TODO: is curently aiming at target
    return Util.epislonEquals(io.current_wrist_angle, io.target_wrist_angle, ShooterConstatnts.wristTolerance) &&
    Util.epislonEquals(io.current_flywheel_speed, io.target_flywheel_speed, ShooterConstatnts.flywheelTolerance);
  }

  public boolean hasNote(){
    //TODO: not is in holding postion
    return io.has_note;
  }


  //set methods
  public void setAngle(double goal){
    angleControler.setGoal(goal);
  }

  public void setTarget(ShootTarget wantedTarget){
    var alliance = DriverStation.getAlliance();
    if(DriverStation.Alliance.Red == alliance.get()){
      if(wantedTarget == ShootTarget.SPEAKER){
        io.target = redSpeaker;
      } else {
        io.target = redAmp;
      }
    } else {
      if(wantedTarget == ShootTarget.SPEAKER){
        io.target = blueSpeaker;
      } else {
        io.target = blueAmp;
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
    public Pose3d target;
    public double target_flywheel_speed;
    public double current_flywheel_speed;
    public double target_wrist_angle;
    public double current_wrist_angle;
    public ShootMode mode = ShootMode.IDLE;
    public double roller_speed;
    public boolean has_note;
  }
}
