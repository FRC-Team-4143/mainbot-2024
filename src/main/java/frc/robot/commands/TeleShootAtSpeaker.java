// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;
import frc.robot.subsystems.ShooterSubsystem.ShootTarget;
import frc.robot.subsystems.SwerveDrivetrain;

public class TeleShootAtSpeaker extends Command {
  /** Creates a new ShootAtTarget. */
  boolean shot_note_;
  public TeleShootAtSpeaker() {
    addRequirements(ShooterSubsystem.getInstance());
    addRequirements(SwerveDrivetrain.getInstance());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterSubsystem.getInstance().setTarget(ShootTarget.SPEAKER);
    ShooterSubsystem.getInstance().setShootMode(ShootMode.PROFILE);
    //SwerveDrivetrain.getInstance().setDriveMode(SwerveDrivetrain.DriveMode.TARGET);
    shot_note_ = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!ShooterSubsystem.getInstance().hasNote() && !shot_note_){
      CommandScheduler.getInstance().schedule(new TeleRearPickup());
    } else if (ShooterSubsystem.getInstance().hasNote() && ShooterSubsystem.getInstance().isTargetLocked()){
      ShooterSubsystem.getInstance().setRollerFeed();
      shot_note_ = true;
    } else {
      ShooterSubsystem.getInstance().rollerStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSubsystem.getInstance().flyWheelStop();
    SwerveDrivetrain.getInstance().setDriveMode(SwerveDrivetrain.DriveMode.FIELD_CENTRIC);
    ShooterSubsystem.getInstance().setShootMode(ShootMode.IDLE);
    ShooterSubsystem.getInstance().rollerStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
