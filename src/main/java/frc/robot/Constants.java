// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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
    public static final Transform3d SHOOTER_OFFSET = new Transform3d(SHOOTER_X_OFFSET, SHOOTER_Y_OFFSET, SHOOTER_Z_OFFSET, new Rotation3d(0, 0, 0)); //TODO: figure out constants

    // Flywheel constants
    public static final int TOP_FLYWHEEL_MOTOR_ID = 10;
    public static final int BOT_FLYWHEEL_MOTOR_ID = 11;
    public static final double FLYWHEEL_IDLE_SPEED = 0.0;
    public static final double FLYWHEEL_TOLERANCE = 0.1;
    public static final double NOTE_EXIT_VELOCITY = 8.0;

    // Wrist constants
    public static final int WRIST_MOTOR_ID = 12;
    public static final int WRIST_ENCODER_ID = 0;
    public static final double WRIST_ANGLE_MAX = 0;
    public static final double WRIST_ANGLE_MIN = 0;
    public static final double WRIST_CONTROLLER_P = 13.0;
    public static final double WRIST_CONTROLLER_FF = 0.2;
    public static final double WRIST_TOLERANCE = 0.00349;
    public static final double WRIST_ZERO_ANGLE = 0.2768 * (2 * Math.PI);
    public static final double WRIST_HOME_ANGLE = 0.78539;

    // Roller constants
    public static final int ROLLER_MOTOR_ID = 13;
    public static final double ROLLER_SPEED = 1.0;

    public static final double YAW_TOLERANCE = 0.034; // 2 Degress for Testing
  }

  // IDs Range from 20 - 29
  public static class PickupConstants {
    public static final double ROLLER_FORWARD = 1.0;
    public static final double ROLLER_REVERSE = -0.5;
    public static final int ROLLER_AMP_LIMIT = 40;
    public static final PickupSettings SHOOTER_PICKUP = new Constants().new PickupSettings(20, true);
    public static final PickupSettings MAILMAN_PICKUP = new Constants().new PickupSettings(21, true);
  }

  public class PickupSettings {
    public PickupSettings(int ID, boolean invert) {
      ROLLER_MOTOR_ID = ID;
      ROLLER_MOTOR_INVERTED = invert;
    }

    public final int ROLLER_MOTOR_ID;
    public final boolean ROLLER_MOTOR_INVERTED;
  }

  public static class MailmanConstants {
    public static final int ELEVATOR_MOTOR_ID = 31;
    public static final int DROPPER_MOTOR_ID = 32;
    public static final double AMP_HEIGHT = 0;
    public static final double TRAP_HEIGHT = 0;
    public static final double HEIGHT_CONTROLLER_P = 0.0;
    public static final double HEIGHT_CONTROLLER_I = 0.0;
    public static final double HEIGHT_CONTROLLER_D = 0.0;
    public static final TrapezoidProfile.Constraints HEIGHT_CONTROLLER_CONSTRAINT = new TrapezoidProfile.Constraints(0,0);
    public static final double DROPPER_IN_SPEED = 0.0;
    public static final double DROPPER_OUT_SPEED = 0.0;
    
  }
}
