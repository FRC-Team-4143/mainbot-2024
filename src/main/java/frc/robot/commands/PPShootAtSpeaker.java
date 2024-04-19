// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;
import frc.robot.subsystems.ShooterSubsystem.ShootTarget;

public class PPShootAtSpeaker extends Command {

  boolean has_shot_note_ = false;
  boolean stop_after_first_ = false;

  /** Creates a new PPShootAtSpeaker. */
  public PPShootAtSpeaker(boolean stop_after_first) {
    // Use addRequirements() here to declare subsystem dependencies.
    stop_after_first_ = stop_after_first;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterSubsystem.getInstance().setTarget(ShootTarget.SPEAKER);
    SwerveDrivetrain.getInstance().setDriveMode(SwerveDrivetrain.DriveMode.AUTONOMOUS_TARGET);
    ShooterSubsystem.getInstance().setShootMode(ShootMode.TARGET);

    has_shot_note_ = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!ShooterSubsystem.getInstance().hasNote()) {
      ShooterSubsystem.getInstance().setRollerFeed();
    } else if (ShooterSubsystem.getInstance().hasNote() && ShooterSubsystem.getInstance().isTargetLocked()) {
      ShooterSubsystem.getInstance().setRollerLaunch();
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
    return has_shot_note_ && (ShooterSubsystem.getInstance().getShootMode() != ShootMode.TARGET
        || (stop_after_first_ && !ShooterSubsystem.getInstance().hasNote()));
  }
}
