// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MailmanSubsystem;
import frc.robot.subsystems.MailmanSubsystem.HeightTarget;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;
import frc.robot.subsystems.ShooterSubsystem.ShootTarget;

public class OverrideShootAtSpeaker extends Command {
  /** Creates a new ShootAtTarget. */
  boolean shot_note_;
  public OverrideShootAtSpeaker() {
    addRequirements(ShooterSubsystem.getInstance());
    addRequirements(MailmanSubsystem.getInstance());
    addRequirements(PickupSubsystem.getMailmanInstance());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    MailmanSubsystem.getInstance().setHeight(HeightTarget.HOME);
    ShooterSubsystem.getInstance().setTarget(ShootTarget.SPEAKER);
    ShooterSubsystem.getInstance().setShootMode(ShootMode.PROFILE);
    shot_note_ = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ShooterSubsystem.getInstance().isOverrideTargetLocked()){
      ShooterSubsystem.getInstance().setRollerFeed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSubsystem.getInstance().setShootMode(ShootMode.IDLE);
    ShooterSubsystem.getInstance().rollerStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
