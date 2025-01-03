// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem.ShootTarget;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.AutoEnableDefaults;
import frc.robot.commands.climberSequence.PresetState;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.ShooterSubsystem;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    static SwerveDrivetrain swerve_drivetrain_ = SwerveDrivetrain.getInstance();
    static PoseEstimator pose_estimator_ = PoseEstimator.getInstance();

    @Override
    public void robotInit() {
        m_robotContainer = RobotContainer.getInstance();
        AutoManager.getInstance();
        OI.configureBindings();
    }

    @Override
    public void robotPeriodic() {
        // Call the scheduler so that commands work for buttons
        CommandScheduler.getInstance().run();

        // tell the subsystems to output telemetry to smartdashboard
        m_robotContainer.outputTelemetry();
    }

    @Override
    public void disabledInit() {
        SwerveDrivetrain.getInstance().setDriveMode(DriveMode.FIELD_CENTRIC);
        if (ClimberSubsystem.getInstance().getTargetHeight() == ClimberConstants.CLIMB_HEIGHT) {
            CommandScheduler.getInstance().schedule(new PresetState().ignoringDisable(true));
        }
    }

    @Override
    public void disabledPeriodic() {
        updateDriverPrespective();
    }

    @Override
    public void autonomousInit() {

        swerve_drivetrain_.setDriveMode(DriveMode.AUTONOMOUS);
        m_autonomousCommand = AutoManager.getInstance().getAutonomousCommand();

        ShooterSubsystem.getInstance().setDefaultCommand(new AutoEnableDefaults());

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        ShooterSubsystem.getInstance().removeDefaultCommand();
        PoseEstimator.getInstance().setIgnoreVision(false);
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.reset();
        SwerveDrivetrain.getInstance().setDriveMode(DriveMode.FIELD_CENTRIC);

        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    private void updateDriverPrespective() {
        if (DriverStation.getAlliance().isPresent()) {
            swerve_drivetrain_.setDriverPrespective(
                    DriverStation.getAlliance().get() == Alliance.Red
                            ? swerve_drivetrain_.redAlliancePerspectiveRotation
                            : swerve_drivetrain_.blueAlliancePerspectiveRotation);
            ShooterSubsystem.getInstance().setTarget(ShootTarget.SPEAKER);
        }
    }

}
