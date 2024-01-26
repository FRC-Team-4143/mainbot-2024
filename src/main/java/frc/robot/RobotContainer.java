// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.subsystem.SubsystemManager;
import frc.robot.subsystems.*;


public class RobotContainer extends SubsystemManager {
  private static RobotContainer instance;
  public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }


  
  public RobotContainer() {
    // !!!!!! ALL SUBSYSTEMS MUST BE REGISTERED HERE TO RUN !!!!!!!
    subsystems.add(ExampleSubsystem.getInstance());
    subsystems.add(SwerveDrivetrain.getInstance());
    subsystems.add(PoseEstimator.getInstance());



    // !!!!! LEAVE THIS LINE AS THE LAST LINE IN THE CONSTRUCTOR !!!!!!
    completeRegistration();
  }

  
}
