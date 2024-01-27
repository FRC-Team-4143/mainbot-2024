// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.subsystem.SubsystemManager;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer extends SubsystemManager {
  private static RobotContainer instance;
  public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }


  /* Setting up bindings for necessary control of the swerve drive platform */
  CommandXboxController driverJoystick = new CommandXboxController(0); // My joystick

  public RobotContainer() {
    // !!!!!! ALL SUBSYSTEMS MUST BE REGISTERED HERE TO RUN !!!!!!!
    // subsystems.add(ExampleSubsystem.getInstance());
    subsystems.add(SwerveDrivetrain.getInstance());
    // subsystems.add(PoseEstimator.getInstance());


    // !!!!! LEAVE THIS LINE AS THE LAST LINE IN THE CONSTRUCTOR !!!!!!
    reset();
    completeRegistration();
  }

  /*
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
   */

  public double getDriverJoystickLeftX() {
    return driverJoystick.getLeftX();
  }
  public double getDriverJoystickRightX() {
    return driverJoystick.getRightX();
  }
  public double getDriverJoystickLeftY() {
    return driverJoystick.getLeftY();
  }
  public double getDriverJoystickRightY() {
    return driverJoystick.getRightY();
  }
}
