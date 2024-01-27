// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

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
    public String[] CANbusName = {"rio", "rio", "rio2", "rio"};

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
  
  public static class ShooterConstatnts {
    // Flywheel constants
    public static final int TOP_FLYWHEEL_MOTOR_ID = 10;
    public static final int BOT_FLYWHEEL_MOTOR_ID = 11;
        public static final double FLYWHEEL_IDLE_SPEED = 0;
            public static final double FLYWHEEL_TOLERANCE = 0.1;

    // Wrist constants
    public static final int WRIST_MOTOR_ID = 12;
    public static final int WRIST_ENCODER_ID = 0;
    public static final double WRIST_ANGLE_MAX = 0;
    public static final double WRIST_ANGLE_MIN = 0;
    public static final double WRIST_CONTROLLER_P = 0.0;
    public static final double WRIST_CONTROLLER_I = 0.0;
    public static final double WRIST_CONTROLLER_D = 0.0;
    public static final double WRIST_TOLERANCE = 0.00349;
    public static final TrapezoidProfile.Constraints WRIST_CONTROLLER_CONSTRAINT 
      = new TrapezoidProfile.Constraints(0, 0);

    // Roller constants
    public static final int ROLLER_MOTOR_ID = 13;
    public static final double ROLLER_SPEED = 0;
  }
}
