package frc.lib.swerve.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import frc.lib.swerve.SwerveModuleConstantsFactory;
import frc.lib.swerve.SwerveModuleConstants.SwerveModuleSteerFeedbackType;
import frc.lib.swerve.SwerveDrivetrainConstants;
import frc.lib.swerve.SwerveModuleConstants;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveDrivetrain;

public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot
    // The steer motor uses MotionMagicVoltage control
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
    // When using closed-loop control, the drive motor uses:
    // - VelocityVoltage, if DrivetrainConstants.SupportsPro is false (default)
    // - VelocityTorqueCurrentFOC, if DrivetrainConstants.SupportsPro is true
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.0).withKI(0).withKD(0)
        .withKS(0).withKV(0.01).withKA(0);

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    private static final double kSpeedAt12VoltsMps = 6.0;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5;
    
    private static final double kDriveGearRatio = 6.21;  //7.363636364;
    private static final double kSteerGearRatio = 150./7.; //15.42857143;
    private static final double kWheelRadiusInches = 2.167; // Estimated at first, then fudge-factored to make odom match record

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = false; //true;

    private static final int kPigeonId = 0;
    private static final double frameWidth = 19.0;
    private static final double frameLength = 18.0;


    // These are only used for simulation
    private static double kSteerInertia = 0.00001;
    private static double kDriveInertia = 0.001;

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withSupportsPro(false);
            //.withSupportsPro(true);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            //.withFeedbackSource(SwerveModuleSteerFeedbackType.FusedCANcoder)
            .withFeedbackSource(SwerveModuleSteerFeedbackType.None)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


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
