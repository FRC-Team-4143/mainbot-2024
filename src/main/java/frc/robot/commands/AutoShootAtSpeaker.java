// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.management.timer.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutoShootAtSpeaker extends Command {

  boolean has_shot_note_ = false;

  /** Creates a new AutoShootAtSpeaker. */
  public AutoShootAtSpeaker() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveDrivetrain.getInstance().setDriveMode(SwerveDrivetrain.DriveMode.TARGET);
    has_shot_note_ = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!ShooterSubsystem.getInstance().hasNote()) {
      ShooterSubsystem.getInstance().setRollerFeed();
    } else if(ShooterSubsystem.getInstance().hasNote() && ShooterSubsystem.getInstance().isOverrideTargetLocked()) { //is target lock normaly 
      ShooterSubsystem.getInstance().setRollerFeed();
      has_shot_note_ = true;
    } else {
      ShooterSubsystem.getInstance().rollerStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveDrivetrain.getInstance().setDriveMode(SwerveDrivetrain.DriveMode.AUTONOMOUS);
    ShooterSubsystem.getInstance().rollerStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return has_shot_note_ && !ShooterSubsystem.getInstance().hasNote();
  }
}
