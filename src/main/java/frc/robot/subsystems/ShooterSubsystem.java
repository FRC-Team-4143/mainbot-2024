// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.Util;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;

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

  // initialize motors
  private CANSparkFlex top_flywheel_motor_;
  private CANSparkFlex bot_flywheel_motor_;
  private CANSparkFlex wrist_motor_;
  private CANSparkMax roller_motor_; // Motor type tbd

  private AprilTagFieldLayout field_layout_ = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private final Transform3d SPEAKER_TRANSFORM = new Transform3d(0, 0, 1, new Rotation3d(0, 0, 0)); // TODO: figure out
                                                                                                   // transformation
  private final Transform3d AMP_TRANSFORM = new Transform3d(0, 0, -1, new Rotation3d(0, 0, 0)); // TODO: figure out
                                                                                                // transformation

  private final Pose3d BLUE_SPEAKER = field_layout_.getTagPose(7).get().transformBy(SPEAKER_TRANSFORM);
  private final Pose3d RED_SPEAKER = field_layout_.getTagPose(4).get().transformBy(SPEAKER_TRANSFORM);
  private final Pose3d BLUE_AMP = field_layout_.getTagPose(6).get().transformBy(AMP_TRANSFORM);
  private final Pose3d RED_AMP = field_layout_.getTagPose(5).get().transformBy(AMP_TRANSFORM);

  private ProfiledPIDController angle_controller_;

  private StructPublisher<Pose3d> target_pub;


  public enum ShootTarget {
    SPEAKER,
    AMP
  }

  public enum ShootMode {
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
    angle_controller_ = new ProfiledPIDController(ShooterConstants.ANGLE_CONTROLLER_P,
        ShooterConstants.ANGLE_CONTROLLER_I, ShooterConstants.ANGLE_CONTROLLER_D,
        ShooterConstants.ANGLE_CONTROLLER_CONSTRAINT);
    top_flywheel_motor_ = new CANSparkFlex(ShooterConstants.TOP_FLYWHEEL_MOTOR_ID,
        CANSparkLowLevel.MotorType.kBrushless);
    bot_flywheel_motor_ = new CANSparkFlex(ShooterConstants.BOT_FLYWHEEL_MOTOR_ID,
        CANSparkLowLevel.MotorType.kBrushless);
    top_flywheel_motor_.follow(bot_flywheel_motor_, true);
    wrist_motor_ = new CANSparkFlex(ShooterConstants.WRIST_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    roller_motor_ = new CANSparkMax(ShooterConstants.ROLLER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    roller_motor_.setInverted(true);
    target_pub = NetworkTableInstance.getDefault().getStructTopic("tag_pose", Pose3d.struct).publish();
    reset();
  }

  // get methods
  public double getShooterAngle() {
    // TODO: return curent angle
    return io_.current_wrist_angle_;
  }

  public boolean isTargetLocked() {
    // TODO: is curently aiming at target
    return Util.epislonEquals(io_.current_wrist_angle_, io_.target_wrist_angle_, ShooterConstants.ANGLE_TOLERANCE) &&
        Util.epislonEquals(io_.current_flywheel_speed_, io_.target_flywheel_speed_,
            ShooterConstants.FLYWHEEL_TOLERANCE);
  }

  public boolean hasNote() {
    // TODO: not is in holding postion
    return io_.has_note_;
  }

  // set methods
  public void setAngle(double goal) {
    angle_controller_.setGoal(goal);
  }

  public void setTarget(ShootTarget target) {
    var alliance = DriverStation.getAlliance();
    if (DriverStation.Alliance.Red == alliance.get()) {
      if (target == ShootTarget.SPEAKER) {
        io_.target_ = RED_SPEAKER;
      } else {
        io_.target_ = RED_AMP;
      }
    } else {
      if (target == ShootTarget.SPEAKER) {
        io_.target_ = BLUE_SPEAKER;
      } else {
        io_.target_ = BLUE_AMP;
      }
    }
  }

  public void setRollerFeed() {
    io_.roller_speed_ = ShooterConstants.ROLLER_SPEED;
  }

  public void setRollerReverse() {
    io_.roller_speed_ = -ShooterConstants.ROLLER_SPEED;
  }

  public void rollerStop() {
    io_.roller_speed_ = 0;
  }

  public void setWristSpeed(double wristSpeed) {
    io_.wrist_speed_ = wristSpeed;
  }

  public void wristStop() {
    io_.wrist_speed_ = 0;
  }

  // TODO: This method should either be rewritten or only used for manual
  // overrides
  // THIS IS ONLY FOR PROTOTYPE TESTING!!!!
  public void setFlyWheelSpeed(double speed) {
    io_.target_flywheel_speed_ = speed;
  }

  public void flyWheelStop() {
    io_.target_flywheel_speed_ = 0;
  }

  // private void updateTargetTransform(Pose3d target_pose) {
  //   io_.target_transform_ = new Transform3d(target_pose.getTranslation(), target_pose.getRotation());
  // }

  private void transformVelocities(Pose3d target_pose) {
    ChassisSpeeds temp_chassis_speed = SwerveDrivetrain.getInstance().getCurrentRobotChassisSpeeds();
    io_.target_transform_ = new Transform3d(target_pose.getTranslation(), target_pose.getRotation());
    Translation3d temp_translation = new Translation3d(temp_chassis_speed.vxMetersPerSecond,
        temp_chassis_speed.vyMetersPerSecond, 0.0);
    temp_translation.rotateBy(io_.target_transform_.getRotation());
    io_.relative_chassis_speed_ = new ChassisSpeeds(temp_translation.getX(), temp_translation.getY(), 0.0);
  }

  private void calculateNoteTravelTime(Pose3d robot_pose, Pose3d target_pose) {
    double temp_distance = robot_pose.getTranslation().getDistance(target_pose.getTranslation());
    io_.note_travel_time_ = temp_distance / ShooterConstants.NOTE_EXIT_VELOCITY; // TODO Find the actual exit velocity

    // Pose3d pose_difference = robot_pose.relativeTo(target_pose);
    // pose_difference.getTranslation().getDistance(robot_pose.toPose2d().getTranslation());
  }

  private void calculateWristAngle(Pose3d robot_pose, Pose3d target_pose, double velocity) {
    robot_pose = robot_pose.transformBy(ShooterConstants.SHOOTER_OFFSET);
    double x = Math.abs(robot_pose.getX() - target_pose.getX());
    double z = Math.abs(robot_pose.getZ() - target_pose.getZ());
    double d = Math.sqrt((x * x) + (z * z));
    double G = 9.81;
    double root = Math.pow(velocity, 4) - G * (G * velocity * velocity + 2 * velocity * z);
    io_.wrist_angle_ = Math.atan2((velocity * velocity) - Math.sqrt(root), G * d);
  }

  private void calculateTargetYaw(Pose2d robot_pose) {
    Pose2d pose_difference = robot_pose.relativeTo(io_.target_.toPose2d());
    io_.target_robot_yaw_ = pose_difference.getTranslation().getAngle();

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
    calculateTargetYaw(PoseEstimator.getInstance().getRobotPose());

  }

  @Override
  /**
   * Inside this function actuator OUTPUTS should be updated from data contained
   * in
   * the PeriodicIO class defined below. There should be little to no logic
   * contained within this function, and no sensors should be read.
   */
  public void writePeriodicOutputs(double timestamp) {
    bot_flywheel_motor_.set(io_.target_flywheel_speed_);
    roller_motor_.set(io_.roller_speed_);
    wrist_motor_.set(io_.wrist_speed_);
    SwerveDrivetrain.getInstance().setTargetRotation(io_.target_robot_yaw_);
    

  }

  @Override
  /**
   * Inside this function telemetry should be output to smartdashboard. The data
   * should be collected out of the PeriodicIO class instance defined below. There
   * should be no sensor information read in this function nor any outputs made to
   * actuators within this function. Only publish to smartdashboard here.
   */
  public void outputTelemetry(double timestamp) {
    SmartDashboard.putNumber("Target Yaw", io_.target_robot_yaw_.getDegrees());
    target_pub.set(io_.target_);
  }

  @Override
  public LogData getLogger() {
    return io_;
  }

  public class ShooterPeriodicIo extends LogData {
    public Pose3d target_ = new Pose3d();
    public double target_flywheel_speed_;
    public double current_flywheel_speed_;
    public double target_wrist_angle_;
    public double current_wrist_angle_;
    public ShootMode target_mode_ = ShootMode.IDLE;
    public double roller_speed_;
    public boolean has_note_;
    public double wrist_speed_;
    public double wrist_angle_;
    public Rotation2d target_robot_yaw_ = new Rotation2d();
    public double note_travel_time_;
    public Transform3d target_transform_;
    public ChassisSpeeds relative_chassis_speed_;
  }
}
