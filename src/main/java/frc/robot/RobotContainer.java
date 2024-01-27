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
