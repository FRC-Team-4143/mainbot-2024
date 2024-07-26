// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.MailmanSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.MailmanSubsystem.HeightTarget;
import frc.robot.subsystems.PickupSubsystem;

public class DemoShoot extends Command {
  /** Creates a new ShootAtTarget. */
  boolean shot_note_;
  int ready_count_;

  public DemoShoot() {
    addRequirements(ShooterSubsystem.getInstance());
    addRequirements(SwerveDrivetrain.getInstance());
    addRequirements(MailmanSubsystem.getInstance());
    addRequirements(PickupSubsystem.getShooterInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    MailmanSubsystem.getInstance().setHeight(HeightTarget.HOME);
    ShooterSubsystem.getInstance().setShootMode(ShootMode.DEMO);
    shot_note_ = false;
    ready_count_ = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!ShooterSubsystem.getInstance().hasNote() && !shot_note_) {
      CommandScheduler.getInstance().schedule(new TeleRearPickupIndex().withTimeout(1));
    } else if (ShooterSubsystem.getInstance().hasNote()) {
      ready_count_++;
    }
    if (ready_count_ >= 25) {
      ShooterSubsystem.getInstance().setRollerLaunch();
      shot_note_ = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveDrivetrain.getInstance().setDriveMode(SwerveDrivetrain.DriveMode.FIELD_CENTRIC);
    ShooterSubsystem.getInstance().rollerStop();
    ShooterSubsystem.getInstance().setShootMode(ShootMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
