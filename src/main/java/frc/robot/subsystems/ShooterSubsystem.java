// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class ShooterSubsystem extends Subsystem {
  
  //initialize motors
  private CANSparkFlex top_flywheel_motor;
  private CANSparkFlex bot_flywheel_motor;
  private CANSparkFlex pivot_motor;
  private CANSparkFlex roller_motor; //Motor type tbd

  private AprilTagFieldLayout field_layout;
  
  private final Rotation3d zero_rotation = new Rotation3d(0, 0, 0);
  private final Transform3d speaker_transform = new Transform3d(0, 0, 1, zero_rotation); //TODO: figure out transformation
  private final Transform3d amp_transform = new Transform3d(0, 0, -1, zero_rotation); //TODO: figure out transformation

  private final Pose3d blue_speaker = field_layout.getTagPose(7).get().transformBy(speaker_transform);
  private final Pose3d red_speaker = field_layout.getTagPose(4).get().transformBy(speaker_transform);
  private final Pose3d blue_amp = field_layout.getTagPose(6).get().transformBy(amp_transform);
  private final Pose3d red_amp = field_layout.getTagPose(5).get().transformBy(amp_transform);

  private ProfiledPIDController angle_controler;

  public enum ShootTarget{
    SPEAKER, 
    AMP
  }

  // Singleton pattern
  private static ShooterSubsystem instance = null;

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  /**
   * 
   */
  private ShooterPeriodicIo io;

  /**
   * Constructor for the example subsystem. The constructor should create all
   * instances of the required hardware as well as the PeriodicIO class defined
   * below. This should not attempt to configure any of the hardware as that
   * should be done in the reset() function.
   */
  public ShooterSubsystem() {
    io = new ShooterPeriodicIo();

    angle_controler = new ProfiledPIDController(Constants.ShooterSubsystemConstatnts.kAngleControlerP, 
                                                Constants.ShooterSubsystemConstatnts.kAngleControlerI, 
                                                Constants.ShooterSubsystemConstatnts.kAngleControlerD, 
                                                Constants.ShooterSubsystemConstatnts.angleControlerConstraint);
    top_flywheel_motor = new CANSparkFlex(Constants.ShooterSubsystemConstatnts.topFlywheelMotorID, CANSparkLowLevel.MotorType.kBrushless);
    bot_flywheel_motor = new CANSparkFlex​(Constants.ShooterSubsystemConstatnts.botFlywheelMotorID, CANSparkLowLevel.MotorType.kBrushless);
    pivot_motor = new CANSparkFlex​(Constants.ShooterSubsystemConstatnts.pivotMotorID, CANSparkLowLevel.MotorType.kBrushless);
    roller_motor = new CANSparkFlex​(Constants.ShooterSubsystemConstatnts.rollerMotorID, CANSparkLowLevel.MotorType.kBrushless);
    reset();
  }
  
  //get methods
  public double getShooterAngle(){
    //TODO: return curent angle
    return 0;
  }

  public boolean isTargetLocked(){
    //TODO: is curently aming at target
    return false;
  }

  public boolean hasNote(){
    //TODO: not is in holding postion
    return false;
  }


  //set methods
  public void setAngle(double goal){
    angle_controler.setGoal(goal);
  }

  public void setTarget(ShootTarget desiredShootTarget){
    var alliance = DriverStation.getAlliance();
    if(DriverStation.Alliance.Red == alliance.get()){
      if(desiredShootTarget == ShootTarget.SPEAKER){
        io.target = red_speaker;
      } else {
        io.target = red_amp;
      }
    } else {
      if(desiredShootTarget == ShootTarget.SPEAKER){
        io.target = blue_speaker;
      } else {
        io.target = blue_amp;
      }
    }
  }

  public void setFeedSpeed(){

  } 

  public void setFlyWheelSpeed(){

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
  }
}
