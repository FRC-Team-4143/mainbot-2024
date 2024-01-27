/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

import frc.lib.subsystem.Subsystem;
import frc.lib.swerve.*;
import frc.lib.swerve.SwerveRequest.SwerveControlRequestParameters;
import frc.lib.swerve.generated.TunerConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API.
 * <p>
 * This class handles the kinematics, configuration, and odometry of a
 * swerve drive utilizing CTR Electronics devices. We recommend
 * that users use the Swerve Mechanism Generator in Tuner X to create
 * a template project that demonstrates how to use this class.
 * <p>
 * This class will construct the hardware devices internally, so the user
 * only specifies the constants (IDs, PID gains, gear ratios, etc).
 * Getters for these hardware devices are available.
 * <p>
 * If using the generator, the order in which modules are constructed is
 * Front Left, Front Right, Back Left, Back Right. This means if you need
 * the Back Left module, call {@code getModule(2);} to get the 3rd index
 * (0-indexed) module, corresponding to the Back Left module.
 */
public class SwerveDrivetrain extends Subsystem {
    private static SwerveDrivetrain instance;

    public static SwerveDrivetrain getInstance() {
        if (instance == null) {
            instance = new SwerveDrivetrain(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft,
                    TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        }
        return instance;
    }

    // Drive Mode Selections
    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        TARGET,
        AUTONOMOUS
    }

    DriveMode drive_mode = DriveMode.ROBOT_CENTRIC;
    SwerveRequest.FieldCentric field_centric = new SwerveRequest.FieldCentric().withIsOpenLoop(true)
            .withDeadband(Constants.DrivetrainConstants.MaxSpeed * 0.1)
            .withRotationalDeadband(Constants.DrivetrainConstants.MaxAngularRate * 0.1);
    SwerveRequest.RobotCentric robot_centric = new SwerveRequest.RobotCentric().withIsOpenLoop(true)
            .withDeadband(Constants.DrivetrainConstants.MaxSpeed * 0.1)
            .withRotationalDeadband(Constants.DrivetrainConstants.MaxAngularRate * 0.1);

    // Robot Hardware
    protected final Pigeon2 pigeon_imu;
    protected final SwerveModule[] swerve_modules;

    // Subsystem data class
    protected PeriodicIo io;

    // Drivetrain config
    protected final SwerveDriveKinematics kinematics;
    protected final Translation2d[] module_locations;

    private final SwerveRequest.ApplyChassisSpeeds auto_request = new SwerveRequest.ApplyChassisSpeeds();
    protected SwerveRequest request_to_apply = new SwerveRequest.Idle();
    protected SwerveControlRequestParameters request_parameters = new SwerveControlRequestParameters();

    private StructArrayPublisher<SwerveModuleState> current_state_pub, requested_state_pub;
    private StructPublisher<Rotation2d> orient_pub;

    /**
     * Constructs a SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * @param driveTrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public SwerveDrivetrain(
            SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants... modules) {

        // make new io instance
        io = new PeriodicIo();

        // Setup the Pigeon IMU
        pigeon_imu = new Pigeon2(driveTrainConstants.Pigeon2Id, driveTrainConstants.CANbusName[0]);
        pigeon_imu.optimizeBusUtilization();

        // Begin configuring swerve modules
        module_locations = new Translation2d[modules.length];
        swerve_modules = new SwerveModule[modules.length];
        io.module_positions = new SwerveModulePosition[modules.length];
        io.current_module_states = new SwerveModuleState[modules.length];
        io.requested_module_states = new SwerveModuleState[modules.length];

        // Construct the swerve modules
        for (int i = 0; i < modules.length; i++) {
            SwerveModuleConstants module = modules[i];
            swerve_modules[i] = new SwerveModule(module, driveTrainConstants.CANbusName[i],
                    driveTrainConstants.SupportsPro);
            module_locations[i] = new Translation2d(module.LocationX, module.LocationY);
            io.module_positions[i] = swerve_modules[i].getPosition(true);
            io.current_module_states[i] = swerve_modules[i].getCurrentState();
            io.requested_module_states[i] = swerve_modules[i].getRequestedState();

        }
        kinematics = new SwerveDriveKinematics(module_locations);

        // NT Publishers
        requested_state_pub = NetworkTableInstance.getDefault()
                .getStructArrayTopic("module_states/requested", SwerveModuleState.struct).publish();
        current_state_pub = NetworkTableInstance.getDefault()
                .getStructArrayTopic("module_states/current", SwerveModuleState.struct).publish();
        orient_pub = NetworkTableInstance.getDefault().getStructTopic("robot_heading", Rotation2d.struct).publish();
    }

    @Override
    public void reset() {
        // 4 signals for each module + 2 for Pigeon2
        for (int i = 0; i < swerve_modules.length; ++i) {
            BaseStatusSignal.setUpdateFrequencyForAll(100, swerve_modules[i].getSignals());
            swerve_modules[i].optimizeCan();
        }
        BaseStatusSignal[] imuSignals = { pigeon_imu.getYaw() };
        BaseStatusSignal.setUpdateFrequencyForAll(100, imuSignals);
        pigeon_imu.optimizeBusUtilization();

        configurePathPlanner();
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        for (int i = 0; i < swerve_modules.length; ++i) {
            io.module_positions[i] = swerve_modules[i].getPosition(true);
            io.current_module_states[i] = swerve_modules[i].getCurrentState();
            io.requested_module_states[i] = swerve_modules[i].getRequestedState();
        }
        io.driver_joystick_leftX = RobotContainer.getInstance().getDriverJoystickLeftX();
        io.driver_joystick_leftY = RobotContainer.getInstance().getDriverJoystickLeftY();
        io.driver_joystick_rightX = RobotContainer.getInstance().getDriverJoystickRightX();

        io.robot_yaw = Rotation2d.fromDegrees(-pigeon_imu.getAngle());
    }

    @Override
    public void updateLogic(double timestamp) {
        switch (drive_mode) {
            case ROBOT_CENTRIC:
                setControl(robot_centric
                        // Drive forward with negative Y (forward)
                        .withVelocityX(-io.driver_joystick_leftY * Constants.DrivetrainConstants.MaxSpeed)
                        // Drive left with negative X (left)
                        .withVelocityY(-io.driver_joystick_leftX * Constants.DrivetrainConstants.MaxSpeed)
                        // Drive counterclockwise with negative X (left)
                        .withRotationalRate(-io.driver_joystick_rightX * Constants.DrivetrainConstants.MaxAngularRate));
                break;
            case FIELD_CENTRIC:
                setControl(field_centric
                        // Drive forward with negative Y (forward)
                        .withVelocityX(-io.driver_joystick_leftY * Constants.DrivetrainConstants.MaxSpeed)
                        // Drive left with negative X (left)
                        .withVelocityY(-io.driver_joystick_leftX * Constants.DrivetrainConstants.MaxSpeed)
                        // Drive counterclockwise with negative X (left)
                        .withRotationalRate(-io.driver_joystick_rightX * Constants.DrivetrainConstants.MaxAngularRate));
                break;
            default:
                // yes these dont do anything for auto...
                break;
        }

        /* And now that we've got the new odometry, update the controls */
        request_parameters.currentPose = new Pose2d(0, 0, io.robot_yaw)
                .relativeTo(new Pose2d(0, 0, io.field_relative_offset));
        request_parameters.kinematics = kinematics;
        request_parameters.swervePositions = module_locations;
        request_parameters.updatePeriod = 1.0 / (4.0 * (timestamp - request_parameters.timestamp));
        request_parameters.timestamp = timestamp;
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        request_to_apply.apply(request_parameters, swerve_modules);
    }

    @Override
    public void outputTelemetry(double timestamp) {
        current_state_pub.set(io.current_module_states);
        requested_state_pub.set(io.requested_module_states);

        orient_pub.set(io.robot_yaw);
    }

    /**
     * Configures the PathPlanner AutoBuilder
     */
    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : module_locations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                PoseEstimator.getInstance()::getRobotPose, // Supplier of current robot pose
                PoseEstimator.getInstance()::setRobotOdometry, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(auto_request.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                              // robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        5,
                        driveBaseRadius,
                        new ReplanningConfig(false, false),
                        0.008), // faster period than default

                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return Commands.run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return kinematics.toChassisSpeeds(io.current_module_states);
    }

    /**
     * Takes the current orientation of the robot and makes it X forward for
     * field-relative maneuvers.
     */
    public void seedFieldRelative() {
        io.field_relative_offset = io.robot_yaw;
    }

    /**
     * Applies the specified control request to this swerve drivetrain.
     *
     * @param request Request to apply
     */
    public void setControl(SwerveRequest request) {
        request_to_apply = request;
    }

    /**
     * Zero's this swerve drive's odometry entirely.
     * <p>
     * This will zero the entire odometry, and place the robot at 0,0
     */
    public void tareEverything() {
        for (int i = 0; i < swerve_modules.length; ++i) {
            swerve_modules[i].resetPosition();
            swerve_modules[i].setWheelOffsets();
            io.module_positions[i] = swerve_modules[i].getPosition(true);
        }
    }

    /**
     * Gets the raw value from the Robot IMU
     */
    public Rotation2d getImuYaw() {
        return io.robot_yaw;
    }

    /**
     * Returns the module locations in reference to the center of the robot as an
     * array
     * [FrontLeft, FrontRight, BackLeft, BackRight]
     */
    public SwerveModulePosition[] getModulePositions() {
        return io.module_positions;
    }

    /**
     * Returns the module states of the swerve drive as an array
     * [FrontLeft, FrontRight, BackLeft, BackRight]
     */
    public SwerveModuleState[] getModuleStates() {
        return io.current_module_states;
    }

    /**
     * updates the mode flag thats changes what request is applied to the drive
     * train
     * 
     * @param mode drive to switch to [ROBOT_CENTRIC, FIELD_CENTRIC]
     */
    public void setDriveMode(DriveMode mode) {
        drive_mode = mode;
    }

    @Override
    public LogData getLogger() {
        return io;
    }

    /**
     * Plain-Old-Data class holding the state of the swerve drivetrain.
     * This encapsulates most data that is relevant for telemetry or
     * decision-making from the Swerve Drive.
     */
    public class PeriodicIo extends LogData {
        public SwerveModuleState[] current_module_states, requested_module_states;
        public SwerveModulePosition[] module_positions;

        public Rotation2d field_relative_offset = new Rotation2d();
        public Rotation2d robot_yaw = new Rotation2d();

        public double driver_joystick_leftX = 0.0;
        public double driver_joystick_leftY = 0.0;
        public double driver_joystick_rightX = 0.0;
    }
}
