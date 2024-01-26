/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package frc.lib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Preferences;

/**
 * Swerve Module class that encapsulates a swerve module powered by CTR
 * Electronics devices.
 * <p>
 * This class handles the hardware devices and configures them for
 * swerve module operation using the Phoenix 6 API.
 * <p>
 * This class constructs hardware devices internally, so the user
 * only specifies the constants (IDs, PID gains, gear ratios, etc).
 * Getters for these hardware devices are available.
 */
public class SwerveModule {
    private final TalonFX m_driveMotor;
    private final TalonFX m_steerMotor;
    //private final AnalogEncoder m_encoder;

    private final StatusSignal<Double> m_drivePosition;
    private final StatusSignal<Double> m_driveVelocity;
    private final StatusSignal<Double> m_steerPosition;
    private final StatusSignal<Double> m_steerVelocity;
    private final BaseStatusSignal[] m_signals;
    private final double m_driveRotationsPerMeter;
    private final double m_couplingRatioDriveRotorToCANcoder;

    private final double m_speedAt12VoltsMps;
    private final boolean m_supportsPro;

    private final int m_steerMotorID;
    //private final int m_encoderID;

    private final MotionMagicVoltage m_angleSetter = new MotionMagicVoltage(0);
    private final VelocityTorqueCurrentFOC m_velocityTorqueSetter = new VelocityTorqueCurrentFOC(0);
    private final VelocityVoltage m_velocityVoltageSetter = new VelocityVoltage(0);
    private final VoltageOut m_voltageOpenLoopSetter = new VoltageOut(0);
    private final PositionVoltage m_voltagePosition = new PositionVoltage(0);
    private double angleOffset;

    private SwerveModulePosition m_internalState = new SwerveModulePosition();


    /**
     * Construct a SwerveModule with the specified constants.
     *
     * @param constants   Constants used to construct the module
     * @param canbusName  The name of the CAN bus this module is on
     * @param supportsPro True if the devices are licensed to use Pro features
     */
    public SwerveModule(SwerveModuleConstants constants, String canbusName, boolean supportsPro) {
        m_driveMotor = new TalonFX(constants.DriveMotorId, canbusName);
        m_steerMotor = new TalonFX(constants.SteerMotorId, canbusName);
        m_steerMotorID = constants.SteerMotorId;
        //m_encoderID = constants.encoderId;
        //m_encoder = new AnalogEncoder(constants.encoderId);

        //angleOffset = Preferences.getDouble("Module" + m_encoderID, 0);

        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        talonConfigs.Slot0 = constants.DriveMotorGains;
        talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        talonConfigs.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        talonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        talonConfigs.MotorOutput.Inverted = constants.DriveMotorInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        StatusCode response = m_driveMotor.getConfigurator().apply(talonConfigs);
        if (!response.isOK()) {
            System.out
                    .println("Talon ID " + constants.DriveMotorId + " failed config with error " + response.toString());
        }

        /* Undo changes for torqueCurrent */
        // talonConfigs.TorqueCurrent = new TorqueCurrentConfigs(); // try current limit
        // on steer motor RJS
        /* And to current limits */
        // talonConfigs.CurrentLimits = new CurrentLimitsConfigs();

        talonConfigs.Voltage.PeakForwardVoltage = 5;
        talonConfigs.Voltage.PeakReverseVoltage = -5;

        talonConfigs.Slot0 = constants.SteerMotorGains;
        // Modify configuration to use remote CANcoder fused
        switch (constants.FeedbackSource) {
            case RemoteCANcoder:
                talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                talonConfigs.Feedback.FeedbackRemoteSensorID = constants.encoderId;
                talonConfigs.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
                break;
            case FusedCANcoder:
                talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                talonConfigs.Feedback.FeedbackRemoteSensorID = constants.encoderId;
                talonConfigs.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
                break;
            case SyncCANcoder:
                talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
                talonConfigs.Feedback.FeedbackRemoteSensorID = constants.encoderId;
                talonConfigs.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
                break;
            case AnalogEncoder:
            case None:
                talonConfigs.Feedback.SensorToMechanismRatio = constants.SteerMotorGearRatio;
                break;
        }

        talonConfigs.MotionMagic.MotionMagicCruiseVelocity = 100. / constants.SteerMotorGearRatio;
        talonConfigs.MotionMagic.MotionMagicAcceleration = talonConfigs.MotionMagic.MotionMagicCruiseVelocity * 10.;

        talonConfigs.ClosedLoopGeneral.ContinuousWrap = true; // Enable continuous wrap for swerve modules

        talonConfigs.MotorOutput.Inverted = constants.SteerMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        response = m_steerMotor.getConfigurator().apply(talonConfigs);
        if (!response.isOK()) {
            System.out
                    .println("Talon ID " + constants.DriveMotorId + " failed config with error " + response.toString());
        }

        /*
         * CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
         * cancoderConfigs.MagnetSensor.MagnetOffset = constants.CANcoderOffset;
         * response = m_cancoder.getConfigurator().apply(cancoderConfigs);
         * if (!response.isOK()) {
         * System.out.println(
         * "CANcoder ID " + constants.DriveMotorId + " failed config with error " +
         * response.toString());
         * }
         */

        m_drivePosition = m_driveMotor.getPosition().clone();
        m_driveVelocity = m_driveMotor.getVelocity().clone();
        m_steerPosition = m_steerMotor.getPosition().clone();
        m_steerVelocity = m_steerMotor.getVelocity().clone();

        m_signals = new BaseStatusSignal[4];
        m_signals[0] = m_drivePosition;
        m_signals[1] = m_driveVelocity;
        m_signals[2] = m_steerPosition;
        m_signals[3] = m_steerVelocity;

        /* Calculate the ratio of drive motor rotation to meter on ground */
        double rotationsPerWheelRotation = constants.DriveMotorGearRatio;
        double metersPerWheelRotation = 2 * Math.PI * Units.inchesToMeters(constants.WheelRadius);
        m_driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
        m_couplingRatioDriveRotorToCANcoder = constants.CouplingGearRatio;

        /* Make control requests synchronous */
        m_velocityTorqueSetter.UpdateFreqHz = 0;
        m_velocityVoltageSetter.UpdateFreqHz = 0;
        m_voltageOpenLoopSetter.UpdateFreqHz = 0;
        m_angleSetter.UpdateFreqHz = 0;

        /* Get the expected speed when applying 12 volts */
        m_speedAt12VoltsMps = constants.SpeedAt12VoltsMps;

        /* If this supports pro, save it */
        m_supportsPro = supportsPro;
        /* Set initial wheel offset */
        resetToAbsolute();
    }

    /**
     * Gets the state of this module and passes it back as a
     * SwerveModulePosition object with latency compensated values.
     *
     * @param refresh True if the signals should be refreshed
     * @return SwerveModulePosition containing this module's state.
     */
    public SwerveModulePosition getPosition(boolean refresh) {
        if (refresh) {
            /* Refresh all signals */
            m_drivePosition.refresh();
            m_driveVelocity.refresh();
            m_steerPosition.refresh();
            m_steerVelocity.refresh();
        }

        /* Now latency-compensate our signals */
        double drive_rot = m_drivePosition.getValue(); // BaseStatusSignal.getLatencyCompensatedValue(m_drivePosition,
                                                       // m_driveVelocity);
        double angle_rot = m_steerPosition.getValue(); // BaseStatusSignal.getLatencyCompensatedValue(m_steerPosition,
                                                       // m_steerVelocity);
        // getlatencycompensatedvalue is broken for now RJS

        /*
         * Back out the drive rotations based on angle rotations due to coupling between
         * azimuth and steer
         */
        drive_rot -= angle_rot * m_couplingRatioDriveRotorToCANcoder;

        /* And push them into a SwerveModulePosition object to return */
        m_internalState.distanceMeters = drive_rot / m_driveRotationsPerMeter;
        /* Angle is already in terms of steer rotations */
        m_internalState.angle = Rotation2d.fromRotations(angle_rot);

        return m_internalState;
    }

    /**
     * Applies the desired SwerveModuleState to this module.
     *
     * @param state      Speed and direction the module should target
     * @param isOpenLoop True if this should use open-loop control.
     */
    public void apply(SwerveModuleState state, boolean isOpenLoop) {
        var optimized = SwerveModuleState.optimize(state, m_internalState.angle);

        double angleToSetDeg = optimized.angle.getRotations();
        // double angleToSetDeg = state.angle.getRotations(); // optimize appears broken
        // for now RJS
        m_steerMotor.setControl(m_angleSetter.withPosition(angleToSetDeg).withSlot(0));
        // m_steerMotor.setControl(m_voltagePosition.withPosition(angleToSetDeg).withSlot(0));
        SmartDashboard.putNumber("Motor " + m_steerMotorID + " target position", angleToSetDeg);
        SmartDashboard.putNumber("Motor " + m_steerMotorID + " actual position", m_steerPosition.getValue());
        SmartDashboard.putNumber("Motor " + m_steerMotorID + " internalstate position",
                m_internalState.angle.getRotations());
        //SmartDashboard.putNumber("Analog encoder " + m_encoderID + " position", 0);

        double velocityToSet = optimized.speedMetersPerSecond * m_driveRotationsPerMeter;

        /*
         * From FRC 900's whitepaper, we add a cosine compensator to the applied drive
         * velocity
         */
        /* To reduce the "skew" that occurs when changing direction */
        double steerMotorError = angleToSetDeg - m_steerPosition.getValue();
        /* If error is close to 0 rotations, we're already there, so apply full power */
        /*
         * If the error is close to 0.25 rotations, then we're 90 degrees, so movement
         * doesn't help us at all
         */
        double cosineScalar = Math.cos(Units.rotationsToRadians(steerMotorError));
        /*
         * Make sure we don't invert our drive, even though we shouldn't ever target
         * over 90 degrees anyway
         */
        if (cosineScalar < 0.0) {
            cosineScalar = 0.0;
        }
        velocityToSet *= cosineScalar;

        /* Back out the expected shimmy the drive motor will see */
        /* Find the angular rate to determine what to back out */
        double azimuthTurnRps = m_steerVelocity.getValue();
        /* Azimuth turn rate multiplied by coupling ratio provides back-out rps */
        double driveRateBackOut = azimuthTurnRps * m_couplingRatioDriveRotorToCANcoder;
        velocityToSet -= driveRateBackOut;

        if (isOpenLoop) {
            /*
             * Open loop ignores the driveRotationsPerMeter since it only cares about the
             * open loop at the mechanism
             */
            /* But we do care about the backout due to coupling, so we keep it in */
            velocityToSet /= m_driveRotationsPerMeter;
            /* Open loop always uses voltage setter */
            m_driveMotor.setControl(m_voltageOpenLoopSetter.withOutput(velocityToSet / m_speedAt12VoltsMps * 12.0));
        } else {
            /* If we support pro, use the torque request */
            if (m_supportsPro) {
                m_driveMotor.setControl(m_velocityTorqueSetter.withVelocity(velocityToSet));
            } else {
                m_driveMotor.setControl(m_velocityVoltageSetter.withVelocity(velocityToSet));
            }
        }
    }

    /**
     * Gets the last cached swerve module position.
     * This differs from {@link getPosition} in that it will not
     * perform any latency compensation or refresh the signals.
     *
     * @return Last cached SwerveModulePosition
     */
    public SwerveModulePosition getCachedPosition() {
        return m_internalState;
    }

    /**
     * Get the current state of the module.
     * <p>
     * This is typically used for telemetry, as the SwerveModulePosition
     * is used for odometry.
     *
     * @return Current state of the module
     */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(m_driveVelocity.getValue() / m_driveRotationsPerMeter,
                Rotation2d.fromRotations(m_steerPosition.getValue()));
    }

    /**
     * Gets the position/velocity signals of the drive and steer
     *
     * @return Array of BaseStatusSignals for this module in the following order:
     *         0 - Drive Position
     *         1 - Drive Velocity
     *         2 - Steer Position
     *         3 - Steer Velocity
     */
    public BaseStatusSignal[] getSignals() {
        return m_signals;
    }

    /**
     * Resets this module's drive motor position to 0 rotations.
     */
    public void resetPosition() {
        /* Only touch drive pos, not steer */
        m_driveMotor.setPosition(0);
    }

    public void setWheelOffsets() {
        // m_steerMotor.setPosition(0);

        //angleOffset = m_encoder.getAbsolutePosition() * 360.0;
        //yPreferences.setDouble("Module" + m_encoderID, angleOffset);
        resetToAbsolute();

    }

    public void resetToAbsolute() {
        //double absolutePosition = m_encoder.getAbsolutePosition() * 360.0 - angleOffset;
        //m_steerMotor.setPosition(absolutePosition / 360.0);
    }

    /**
     * Gets this module's Drive Motor TalonFX reference.
     * <p>
     * This should be used only to access signals and change configurations that the
     * swerve drivetrain does not configure itself.
     *
     * @return This module's Drive Motor reference
     */
    public TalonFX getDriveMotor() {
        return m_driveMotor;
    }

    /**
     * Gets this module's Steer Motor TalonFX reference.
     * <p>
     * This should be used only to access signals and change configurations that the
     * swerve drivetrain does not configure itself.
     *
     * @return This module's Steer Motor reference
     */
    public TalonFX getSteerMotor() {
        return m_steerMotor;
    }

    public void optimizeCan() {
        // Silence all status signals for less data
        TalonFX.optimizeBusUtilizationForAll(m_driveMotor, m_steerMotor);
    }

}
