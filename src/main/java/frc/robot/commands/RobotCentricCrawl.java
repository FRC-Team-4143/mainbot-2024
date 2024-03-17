// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import frc.robot.subsystems.SwerveDrivetrain;

public class RobotCentricCrawl extends Command {
  static SwerveDrivetrain swerve_drivetrain_ = SwerveDrivetrain.getInstance();
  
  public RobotCentricCrawl() {
    addRequirements(SwerveDrivetrain.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve_drivetrain_.setDriveMode(DriveMode.CRAWL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d r2 = Rotation2d.fromDegrees(OI.getDriverJoystickPOVangle());
    swerve_drivetrain_.setCrabRequest(r2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve_drivetrain_.setDriveMode(DriveMode.FIELD_CENTRIC);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
