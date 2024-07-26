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
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.PickupSubsystem.PickupMode;

public class AutoShootAtSpeaker extends Command {

  boolean has_shot_note_ = false;
  int timeout = 0;
  int counter = 0;

  /** Creates a new AutoShootAtSpeaker. */
  public AutoShootAtSpeaker() {
    addRequirements(ShooterSubsystem.getInstance());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    MailmanSubsystem.getInstance().setHeight(HeightTarget.HOME);
    ShooterSubsystem.getInstance().setTarget(ShootTarget.SPEAKER);
    ShooterSubsystem.getInstance().setShootMode(ShootMode.TARGET);
    SwerveDrivetrain.getInstance().setDriveMode(SwerveDrivetrain.DriveMode.TARGET);
    has_shot_note_ = false;
    timeout = 0;
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ShooterSubsystem.getInstance().hasNote() || PickupSubsystem.getShooterInstance().hasNote()) {
      timeout = 0;
    } else {
      timeout++;
    }
    if (!ShooterSubsystem.getInstance().hasNote()) {
      PickupSubsystem.getShooterInstance().setPickupMode(PickupMode.PICKUP);
      ShooterSubsystem.getInstance().setRollerFeed();
    } else if(ShooterSubsystem.getInstance().hasNote() && ShooterSubsystem.getInstance().isTargetLocked()) { //is target lock normaly
      counter++; 
    } else {
      PickupSubsystem.getShooterInstance().setPickupMode(PickupMode.IDLE);
      ShooterSubsystem.getInstance().rollerStop();
    }

    if(counter > 12){
      ShooterSubsystem.getInstance().setRollerLaunch();
      has_shot_note_ = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PickupSubsystem.getShooterInstance().setPickupMode(PickupMode.PICKUP);
    SwerveDrivetrain.getInstance().setDriveMode(SwerveDrivetrain.DriveMode.AUTONOMOUS);
    ShooterSubsystem.getInstance().rollerStop();
    ShooterSubsystem.getInstance().setShootMode(ShootMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timeout >= 50 || has_shot_note_) && !ShooterSubsystem.getInstance().hasNote() && !PickupSubsystem.getShooterInstance().hasNote();
  }
}
