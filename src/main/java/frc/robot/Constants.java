// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.configs.Slot0Configs;
import frc.lib.swerve.SwerveModuleConstantsFactory;
import frc.lib.swerve.SwerveModuleConstants.SteerFeedbackType;
import frc.lib.swerve.SwerveModule.ClosedLoopOutputType;
import frc.lib.swerve.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
    
  public class DrivetrainConstants {
    // Can bus names for each of the swerve modules
    public static String[] CANbusName = { "CANivore", "CANivore", "CANivore", "CANivore" };

    // Can bus ID for the pigeon
    public static int Pigeon2Id = 0;

    // Both sets of gains need to be tuned to your individual robot
    // The steer motor uses MotionMagicVoltage control
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
    // When using closed-loop control, the drive motor uses:
    // - VelocityVoltage, if DrivetrainConstants.SupportsPro is false (default)
    // - VelocityTorqueCurrentFOC, if DrivetrainConstants.SupportsPro is true
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(7.0).withKI(0.0).withKD(0.0)
        .withKS(2.4).withKV(0.0).withKA(0.0);

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 80.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    private static final double kSpeedAt12VoltsMps = 6.0;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5;
    
    private static final double kDriveGearRatio = 5.14; // Mk4i: 6.12, Mk4: 5.14 
    private static final double kSteerGearRatio = 12.8; // Mk4i: (150.0/7.0), Mk4: 12.8
    private static final double kWheelRadiusInches = 1.6090288; //1.59997; // Estimated at first, then fudge-factored to make odom match record

    private static final boolean kSteerMotorReversed = false;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = false; //true;

    private static final double frameWidth = 19.0;
    private static final double frameLength = 18.0;

    public static final double MaxSpeed = 6; // 6 meters per second desired top speed
    public static final double MaxAngularRate = Math.PI * 2; // Rotation per second max angular velocity
    public static final double CrawlSpeed = 0.4;

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withFeedbackSource(SteerFeedbackType.None) //.withFeedbackSource(SteerFeedbackType.FusedCANcoder) CRH: Removed for AnalogEncoders
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed)
            .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.TorqueCurrentFOC);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 1;
    private static final int kFrontLeftSteerMotorId = 2;
    private static final int kFrontLeftEncoderId = 0;
    private static final double kFrontLeftEncoderOffset = 0;

    private static final double kFrontLeftXPosInches = frameWidth/2;
    private static final double kFrontLeftYPosInches = frameLength/2;
    
    // Front Right
    private static final int kFrontRightDriveMotorId = 3;
    private static final int kFrontRightSteerMotorId = 4;
    private static final int kFrontRightEncoderId = 1;
    private static final double kFrontRightEncoderOffset = 0.0;

    private static final double kFrontRightXPosInches = frameWidth/2.;
    private static final double kFrontRightYPosInches = -frameLength/2;

    // Back Left
    private static final int kBackLeftDriveMotorId = 5;
    private static final int kBackLeftSteerMotorId = 6;
    private static final int kBackLeftEncoderId = 2;
    private static final double kBackLeftEncoderOffset = 0.0;

    private static final double kBackLeftXPosInches = -frameWidth/2.;
    private static final double kBackLeftYPosInches = frameLength/2;

    // Back Right
    private static final int kBackRightDriveMotorId = 7;
    private static final int kBackRightSteerMotorId = 8;
    private static final int kBackRightEncoderId = 3;
    private static final double kBackRightEncoderOffset = 0;

    private static final double kBackRightXPosInches = -frameWidth/2.;
    private static final double kBackRightYPosInches = -frameLength/2;


    public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

  }

  // IDs Range from 10 - 19
  public static class ShooterConstants {
    public static final double SHOOTER_X_OFFSET = -0.3286; // 12.940 in
    public static final double SHOOTER_Y_OFFSET = 0;
    public static final double SHOOTER_Z_OFFSET = 0.4404; // 17.34 in
    public static final Transform3d SHOOTER_OFFSET = new Transform3d(SHOOTER_X_OFFSET, SHOOTER_Y_OFFSET,
        SHOOTER_Z_OFFSET, new Rotation3d(0, 0, 0));

    // Flywheel constants
    public static final int TOP_FLYWHEEL_MOTOR_ID = 10;
    public static final int BOT_FLYWHEEL_MOTOR_ID = 11;
    public static final double FLYWHEEL_IDLE_VOLTAGE = 0.0;
    public static final double FLYWHEEL_TOLERANCE = 30; // TODO: Tune for new PID controller
    public static final double NOTE_EXIT_VELOCITY = (4.0 * 25.4 * Math.PI / 1000.0) * (5252.11 / 60.0) * 0.8; // Linear Shooter Velocity (80% for loss)
    public static final double FLYWHEEL_CONTROLLER_P = 0.001;
    public static final double FLYWHEEL_CONTROLLER_FF = 0.00015;

    public static final InterpolatingDoubleTreeMap DISTANCE_TO_TARGET_OFFSET_MAP() {
      var map = new InterpolatingDoubleTreeMap();
      map.put(0.0, 0.0);
      map.put(1.6, 0.2);
      map.put(3.0, 0.2); // 0.2
      map.put(6.0, 0.45); // 0.6

      return map;
    }

    // Wrist constants
    public static final int WRIST_MOTOR_ID = 12;
    public static final int WRIST_ENCODER_ID = 0;    
    public static final double WRIST_ANGLE_MAX = 0;
    public static final double WRIST_ANGLE_MIN = 0;
    public static final double WRIST_CONTROLLER_P = 13.0;
    public static final double WRIST_CONTROLLER_FF = 0.2;
    public static final double WRIST_TOLERANCE = Math.toRadians(4);
    public static final double WRIST_ZERO_ANGLE = 0.2768 * (2 * Math.PI);
    public static final double WRIST_HOME_ANGLE = 0.22689;
    public static final double WRIST_HANDOFF_ANGLE = 0.1222;
    public static final double WRIST_CLIMB_ANGLE = Math.toRadians(60);

    // Roller constants
    public static final int ROLLER_MOTOR_ID = 13;
    public static final double ROLLER_SPEED = 0.40;

    public static final double YAW_TOLERANCE = Math.toRadians(5);

    // Sensor Constants
    public static final int NOTE_SENSOR_ID = 1;
    public static final double SENSOR_SAMPLE_TIME = 24.0;
    public static final double HAS_NOTE_RANGE = 150.0;
    public static final double NO_NOTE_RANGE = 200.0;
  }

  // IDs Range from 20 - 29
  public static class PickupConstants {
    public static final double ROLLER_FORWARD = 1.0;
    public static final double ROLLER_REVERSE = -0.5;
    public static final int ROLLER_AMP_LIMIT = 40;
    public static final PickupSettings SHOOTER_PICKUP = new Constants().new PickupSettings(20, false, -1);
    public static final PickupSettings MAILMAN_PICKUP = new Constants().new PickupSettings(21, true, -1);
    public static final double HAS_NOTE_RANGE = 0;
    public static final double NO_NOTE_RANGE = 1;
    public static final double SENSOR_SAMPLE_TIME = 50.0;
  }

  public class PickupSettings {
    public PickupSettings(int id, boolean invert, int sense_id) {
      ROLLER_MOTOR_ID = id;
      ROLLER_MOTOR_INVERTED = invert;
      PICKUP_NOTE_SENSOR_ID = sense_id;
    }

    public final int ROLLER_MOTOR_ID;
    public final boolean ROLLER_MOTOR_INVERTED;
    public final int PICKUP_NOTE_SENSOR_ID;
  }

  // IDs Range from 30 - 39
  public static class MailmanConstants {
    public static final int ELEVATOR_MOTOR_ID = 31;
    public static final int DROPPER_MOTOR_ID = 32;
    public static final double AMP_HEIGHT = 63;
    public static final double HOME_HEIGHT = 0;
    public static final double TRAP_HEIGHT = 120;
    public static final double ELEVATOR_CONTROLLER_P = 0.5;
    public static final double ELEVATOR_CONTROLLER_D = 0.0;
    public static final double ELEVATOR_CONTROLLER_MAX_VEL = 0.0;
    public static final double ELEVATOR_CONTROLLER_MAX_ACC = 0.0;
    public static final double DROPPER_IN_SPEED = 0.5;
    public static final double DROPPER_OUT_SPEED = -0.5;

  }

  // IDs range from 40 - 49
  public static class ClimberConstants {
    public static final int LEFT_CLIMBER_MOTOR_ID_ = 40;
    public static final int RIGHT_CLIMBER_MOTOR_ID_ = 41;
  }
}
