// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;

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

  public static final boolean IS_COMP_BOT = Preferences.getBoolean("RobotIsComp", true);

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public class DrivetrainConstants {

    // Can bus names for each of the swerve modules
    public static final String[] MODULE_CANBUS_NAME = { "CANivore", "CANivore", "CANivore", "CANivore" };

    // Can bus ID for the pigeon
    public static final int PIGEON2_ID = 0;

    // Both sets of gains need to be tuned to your individual robot
    // The steer motor uses MotionMagicVoltage control
    private static final Slot0Configs STEER_GAINS = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
    // When using closed-loop control, the drive motor uses:
    // - VelocityVoltage, if DrivetrainConstants.SupportsPro is false (default)
    // - VelocityTorqueCurrentFOC, if DrivetrainConstants.SupportsPro is true
    private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
        .withKP(10.0).withKI(0.0).withKD(0.0) // 7 : updated to 3 RJS
        .withKS(0.0).withKV(0.0).withKA(0.0); // 2.4 : updated to 0 RJS

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double SLIP_CURRENT_AMPS = 80.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    private static final double SPEED_AT_12V_MPS = 5.0;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double COUPLE_RATIO = 3.5;

    private static final double DRIVE_GEAR_RATIO = 6.12; // L3: 6.12, L2: 5.14
    private static final double STEER_GEAR_RATIO = 12.8; // Mk4i: (150.0/7.0), Mk4: 12.8
    private static final double WHEEL_RADIUS_INCH = 1.88; // 1.6090288; // 1.59997; // Estimated at first, then fudge-factored to
                                                               // make odom match record

    private static final boolean STEER_MOTOR_REVERSED = false;
    private static final boolean INVERT_LEFT_DRIVE = false;
    private static final boolean INVERT_RIGHT_DRIVE = false; // true;

    private static final double FRAME_WIDTH = 19.0;
    private static final double FRAME_LENGTH = 18.0;

    public static final double MAX_DRIVE_SPEED = 5; // 6 meters per second desired top speed
    public static final double MAX_DRIVE_ANGULAR_RATE = Math.PI * 2; // Rotation per second max angular velocity
    public static final double CRAWL_DRIVE_SPEED = 0.4;
    public static final double MAX_TARGET_SPEED = 1;

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
        .withSteerMotorGearRatio(STEER_GEAR_RATIO)
        .withWheelRadius(WHEEL_RADIUS_INCH)
        .withSlipCurrent(SLIP_CURRENT_AMPS)
        .withSteerMotorGains(STEER_GAINS)
        .withDriveMotorGains(DRIVE_GAINS)
        .withSpeedAt12VoltsMps(SPEED_AT_12V_MPS)
        .withFeedbackSource(SteerFeedbackType.None) // .withFeedbackSource(SteerFeedbackType.FusedCANcoder) CRH: Removed
                                                    // for AnalogEncoders
        .withCouplingGearRatio(COUPLE_RATIO)
        .withSteerMotorInverted(STEER_MOTOR_REVERSED)
        .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
        .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.TorqueCurrentFOC);

    // Front Left
    private static final int FLD_MOTOR_ID = 1;
    private static final int FLS_MOTOR_ID = 2;
    private static final int FLS_ENCODER_ID = 0;
    private static final double FLS_ENCODER_OFFSET = 0;

    private static final double FL_X_POS_INCH = FRAME_WIDTH / 2;
    private static final double FL_Y_POS_INCH = FRAME_LENGTH / 2;

    // Front Right
    private static final int FRD_MOTOR_ID = 3;
    private static final int FRS_MOTOR_ID = 4;
    private static final int FRS_ENCODER_ID = 1;
    private static final double FRS_ENCODER_OFFSET = 0.0;

    private static final double FR_X_POS_INCH = FRAME_WIDTH / 2.;
    private static final double FR_Y_POS_INCH = -FRAME_LENGTH / 2;

    // Back Left
    private static final int BLD_MOTOR_ID = 5;
    private static final int BLS_MOTOR_ID = 6;
    private static final int BLS_ENCODER_ID = 2;
    private static final double BLS_ENCODER_OFFSET = 0.0;

    private static final double BL_X_POS_INCH = -FRAME_WIDTH / 2.;
    private static final double BL_Y_POS_INCH = FRAME_LENGTH / 2;

    // Back Right
    private static final int BRD_MOTOR_ID = 7;
    private static final int BRS_MOTOR_ID = 8;
    private static final int BRS_ENCODER_ID = 3;
    private static final double BRS_ENCODER_OFFSET = 0;

    private static final double BR_X_POS_INCH = -FRAME_WIDTH / 2.;
    private static final double BR_Y_POS_INCH = -FRAME_LENGTH / 2;

    public static final SwerveModuleConstants FL_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
        FLS_MOTOR_ID, FLD_MOTOR_ID, FLS_ENCODER_ID, FLS_ENCODER_OFFSET, Units.inchesToMeters(FL_X_POS_INCH),
        Units.inchesToMeters(FL_Y_POS_INCH), INVERT_LEFT_DRIVE);
    public static final SwerveModuleConstants FR_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
        FRS_MOTOR_ID, FRD_MOTOR_ID, FRS_ENCODER_ID, FRS_ENCODER_OFFSET, Units.inchesToMeters(FR_X_POS_INCH),
        Units.inchesToMeters(FR_Y_POS_INCH), INVERT_RIGHT_DRIVE);
    public static final SwerveModuleConstants BL_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
        BLS_MOTOR_ID, BLD_MOTOR_ID, BLS_ENCODER_ID, BLS_ENCODER_OFFSET, Units.inchesToMeters(BL_X_POS_INCH),
        Units.inchesToMeters(BL_Y_POS_INCH), INVERT_LEFT_DRIVE);
    public static final SwerveModuleConstants BR_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
        BRS_MOTOR_ID, BRD_MOTOR_ID, BRS_ENCODER_ID, BRS_ENCODER_OFFSET, Units.inchesToMeters(BR_X_POS_INCH),
        Units.inchesToMeters(BR_Y_POS_INCH), INVERT_RIGHT_DRIVE);
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
    public static final double FLYWHEEL_TOLERANCE = 30;
    public static final double NOTE_EXIT_VELOCITY = (4.0 * 25.4 * Math.PI / 1000.0) * (5252.11 / 60.0) * 0.8; // Linear Shooter Velocity (80% for loss)
    public static final double NOTE_EXIT_VELOCITY_PASSING = (4.0 * 25.4 * Math.PI / 1000.0) * (2626.056 / 60.0) * 0.8; // 2387.32 : 250 | 2626.056 : 275 | 300 : 2864.78
    public static final double FLYWHEEL_CONTROLLER_P = 0.0001;
    public static final double FLYWHEEL_CONTROLLER_FF = 0.00016;

    public static final InterpolatingDoubleTreeMap DISTANCE_TO_TARGET_OFFSET_MAP() {
      var map = new InterpolatingDoubleTreeMap();
      map.put(0.0, 0.0);
      map.put(1.6, 0.2);
      map.put(3.0, 0.2);
      map.put(6.0, 0.2);
      return map;
    }

    // Wrist constants
    public static final int WRIST_MOTOR_ID = 12;
    public static final int WRIST_ENCODER_ID = 0;
    public static final double WRIST_ANGLE_MAX = 0;
    public static final double WRIST_ANGLE_MIN = 0;
    public static final double WRIST_CONTROLLER_P = 10.0;
    public static final double WRIST_CONTROLLER_FF = 0.2;
    public static final double WRIST_TOLERANCE = Math.toRadians(4);
    public static final double WRIST_ZERO_ANGLE = 0.289 * (2 * Math.PI); // 0.289 for Comp Bot
    public static final double WRIST_HOME_ANGLE = 0.22689;
    public static final double WRIST_HANDOFF_ANGLE = 0.1222;
    public static final double WRIST_CLIMB_ANGLE = Math.toRadians(60);

    // Roller constants
    public static final int ROLLER_MOTOR_ID = 13;
    public static final double ROLLER_SPEED = 0.40;
    public static final boolean ROLLER_MOTOR_INVERTED = false;

    // Yaw Aiming Tolerance
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
    public static final double SENSOR_SAMPLE_TIME = 50.0;
    public static final PickupSettings SHOOTER_PICKUP = new Constants().new PickupSettings(20, false, 3, 180, 200);
    public static final PickupSettings MAILMAN_PICKUP = new Constants().new PickupSettings(21, true, 2, 105, 130);
  }

  // Pickup Settings Class
  public class PickupSettings {
    public PickupSettings(int id, boolean invert, int sense_id, double has_note_range, double no_note_range) {
      ROLLER_MOTOR_ID = id;
      ROLLER_MOTOR_INVERTED = invert;
      PICKUP_NOTE_SENSOR_ID = sense_id;
      HAS_NOTE_RANGE = has_note_range;
      NO_NOTE_RANGE = no_note_range;
    }

    public final int ROLLER_MOTOR_ID;
    public final boolean ROLLER_MOTOR_INVERTED;
    public final int PICKUP_NOTE_SENSOR_ID;
    public final double HAS_NOTE_RANGE;
    public final double NO_NOTE_RANGE;
  }

  // IDs Range from 30 - 39
  public static class MailmanConstants {
    // Preset Elevator Heights
    public static final double AMP_HEIGHT = 63;
    public static final double HOME_HEIGHT = 0;
    public static final double TRAP_HEIGHT = 120;

    // Elevator Constants
    public static final int ELEVATOR_MOTOR_ID = 31;
    public static final double ELEVATOR_CONTROLLER_P = 0.5;
    public static final double ELEVATOR_CONTROLLER_D = 0.0;
    public static final double ELEVATOR_CONTROLLER_MAX_VEL = 0.0;
    public static final double ELEVATOR_CONTROLLER_MAX_ACC = 0.0;
    public static final double ELEVATOR_HEIGHT_TOLERANCE = 1.0;

    // Dropper Motor Constants
    public static final int DROPPER_MOTOR_ID = 0; // PWM Channel
    public static final double DROPPER_IN_SPEED = 0.5;
    public static final double DROPPER_OUT_SPEED = -1.0;
  }

  // IDs range from 40 - 49
  public static class ClimberConstants {
    public static final int LEFT_CLIMBER_MOTOR_ID_ = 40;
    public static final int RIGHT_CLIMBER_MOTOR_ID_ = 41;
    public static final double CLIMBER_CONTROLLER_P = 0.05;
    public static final double WEIGHTED_CLIMBER_CONTROLLER_P = 0.1;
    public static final double CLIMBER_CONTROLLER_FF = 0.0;
    public static final double WEIGHTED_CLIMBER_CONTROLLER_FF = -0.1;
    public static final double CLIMB_HEIGHT = 5.0;
    public static final double HOME_HEIGHT = 0.0;
    public static final double HALF_HEIGHT = -16.0;
    public static final double MAX_HEIGHT = -32.0;
    public static final double HEIGHT_TOLERANCE = 1.0;
  }
}
