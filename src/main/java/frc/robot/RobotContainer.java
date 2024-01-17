// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.logger.ReflectingLogger;
import frc.lib.logger.Logable.LogData;
import frc.lib.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.swerve.generated.TunerConstants;
import frc.lib.swerve.utility.Telemetry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  final double MaxSpeed = 6; // 6 meters per second desired top speed
  final double MaxAngularRate = Math.PI * 2; // Half a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  CommandXboxController joystick = new CommandXboxController(0); // My joystick

  // Logger instance
  ReflectingLogger<LogData> reflectingLogger;

  // Subsystems
  CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  ExampleSubsystem example = ExampleSubsystem.getInstance();

  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withIsOpenLoop(true).withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1); // I want field-centric
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withIsOpenLoop(true);
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */


  Telemetry logger = new Telemetry(MaxSpeed);

  Pose2d odomStart = new Pose2d(0, 0, new Rotation2d(0, 0));

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()).ignoringDisable(true));
    SmartDashboard.putData("Set wheel offsets",
        drivetrain.runOnce(() -> drivetrain.tareEverything()).ignoringDisable(true));

    // if (Utils.isSimulation()) {
    // drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
    // Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    joystick.pov(90).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(-0.5)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
    joystick.pov(270).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(0.5)));

    // Initialize the logger
    ArrayList<LogData> logs = new ArrayList<>();
    logs.add(example.getLogger());
    reflectingLogger = new ReflectingLogger<>(logs);

    SmartDashboard.putNumber("Robot/X", 0);
    SmartDashboard.putNumber("Robot/Y", 0);
    SmartDashboard.putNumber("Robot/Heading", 0);


    SmartDashboard.putData("Reset robot pose", drivetrain.setInitPose("SimplePath"));
    // Commands.runOnce(() -> {
    //   double robotX = SmartDashboard.getNumber("Robot/X", 0);
    //   double robotY = SmartDashboard.getNumber("Robot/Y", 0);
    //   Rotation2d robotHeading = new Rotation2d(SmartDashboard.getNumber("Robot/Heading", 0));
    //   drivetrain.resetPose(new Pose2d(robotX, robotY, robotHeading));
    // }, drivetrain).ignoringDisable(true));

  }

  public RobotContainer() {
    configureBindings();
  }

  public void runLog(double timestamp) {
  ArrayList<LogData> logs = new ArrayList<>();
    logs.add(example.getLogger());
    reflectingLogger.update(logs, timestamp);
  }

  public Command getAutonomousCommand() {
      Command runAuto = drivetrain.getAutoPath("Tests", "SimplePath");
    return runAuto;
  }
}
