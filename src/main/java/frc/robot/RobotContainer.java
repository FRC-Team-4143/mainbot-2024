// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.subsystem.SubsystemManager;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer extends SubsystemManager {
  /* Setting up bindings for necessary control of the swerve drive platform */
  CommandXboxController joystick = new CommandXboxController(0); // My joystick

  public RobotContainer() {
    // !!!!!! ALL SUBSYSTEMS MUST BE REGISTERED HERE TO RUN !!!!!!!
    subsystems.add(ExampleSubsystem.getInstance());
    // subsystems.add(SwerveDrivetrain.getInstance());



    // !!!!! LEAVE THIS LINE AS THE LAST LINE IN THE CONSTRUCTOR !!!!!!
    completeRegistration();
  }

}
