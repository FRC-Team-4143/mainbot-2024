// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.interpolation.*;
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
  public static class DriveConstants {
    // Can bus names for each of the swerve modules
    public String[] CANbusName = { "rio", "rio", "rio", "rio" };

    // Can bus ID for the pigeon
    public int Pigeon2Id = 0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    public static final int frontLeftEncoder = 0;
    public static final int frontRightEncoder = 1;
    public static final int backLeftEncoder = 2;
    public static final int backRightEncoder = 3;
    public static final double MaxSpeed = 6; // 6 meters per second desired top speed
    public static final double MaxAngularRate = Math.PI * 2; // Half a rotation per second max angular velocity

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
    public static final double FLYWHEEL_TOLERANCE = 5;
    public static final double NOTE_EXIT_VELOCITY = 10.0;
    public static final double FLYWHEEL_CONTROLLER_P = 0.0001;
    public static final double FLYWHEEL_CONTROLLER_FF = 0.00015;

    public static final InterpolatingDoubleTreeMap LINEAR_TO_ANGULAR_VEL_MAP() {
      var map = new InterpolatingDoubleTreeMap();
      map.put(0.0, 0.0);
      map.put(10.0, 98.425);

      return map;
    }

    public static final InterpolatingDoubleTreeMap DISTANCE_TO_EXIT_VEL_MAP() {
      var map = new InterpolatingDoubleTreeMap();
      map.put(0.0, 10.0);
      map.put(3.0, 10.0);
      map.put(6.0, 10.0);

      return map;
    }

    // Wrist constants
    public static final int WRIST_MOTOR_ID = 12;
    public static final int WRIST_ENCODER_ID = 0;
    public static final double WRIST_ANGLE_MAX = 0;
    public static final double WRIST_ANGLE_MIN = 0;
    public static final double WRIST_CONTROLLER_P = 13.0;
    public static final double WRIST_CONTROLLER_FF = 0.2;
    public static final double WRIST_TOLERANCE = 0.00349;
    public static final double WRIST_ZERO_ANGLE = 0.2768 * (2 * Math.PI);
    public static final double WRIST_HOME_ANGLE = 0.22689;
    public static final double WRIST_HANDOFF_ANGLE = 0.1222;
    public static final double WRIST_CLIMB_ANGLE = 1.5708;
    

    // Roller constants
    public static final int ROLLER_MOTOR_ID = 13;
    public static final double ROLLER_SPEED = 0.40;

    public static final double YAW_TOLERANCE = 0.034; // 2 Degress for Testing

    // Sensor Constants
    public static final int NOTE_SENSOR_ID = 1;
    public static final double SENSOR_SAMPLE_TIME = 24.0;
    public static final double HAS_NOTE_RANGE = 200.0;
    public static final double NO_NOTE_RANGE = 350.0;
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
    public PickupSettings(int ID, boolean invert, int SENSE_ID) {
      ROLLER_MOTOR_ID = ID;
      ROLLER_MOTOR_INVERTED = invert;
      PICKUP_NOTE_SENSOR_ID = SENSE_ID;
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
    public static final double TRAP_HEIGHT = 105;
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
