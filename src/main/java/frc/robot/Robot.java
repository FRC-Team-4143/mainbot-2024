// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final boolean UseLimelight = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    // CameraServer.startAutomaticCapture();

    // m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);

    SignalLogger.start();
  }

  @Override
  public void robotPeriodic() {
    // Call the scheduler so that commands work for buttons
    CommandScheduler.getInstance().run();

    // tell the subsystems to output telemetry to smartdashboard
    m_robotContainer.outputTelemetry();

    // Integrate limelight pose data
    if (UseLimelight) {
      // var lastResult =
      // LimelightHelpers.getLatestResults("limelight").targetingResults;

      // Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

      // if (lastResult.valid) {
      // m_robotContainer.drivetrain.addVisionMeasurement(llPose,
      // Timer.getFPGATimestamp());
      // }

    }
  }

  @Override
  public void disabledInit() {
    m_robotContainer.stopLog();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.initLogfile();
    m_autonomousCommand = AutoManager.getInstance().getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    m_robotContainer.initLogfile();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
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
}
