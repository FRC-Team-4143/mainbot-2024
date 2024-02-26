// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.MailmanSubsystem;
import frc.robot.subsystems.MailmanSubsystem.HeightTarget;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.PickupSubsystem.PickupMode;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;

public class HandoffToShooter extends Command {
    static ShooterSubsystem shooter_ = ShooterSubsystem.getInstance();
    static PickupSubsystem pickup_front_ = PickupSubsystem.getMailmanInstance();
    static MailmanSubsystem mailman_ = MailmanSubsystem.getInstance();

    boolean seen_note_ = false;

  /** Creates a new HandoffToShooter. */
  public HandoffToShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    seen_note_ = false;
    if (!shooter_.hasNote()){
      mailman_.setHeight(HeightTarget.HOME);
      shooter_.setShootMode(ShootMode.RECEIVE);
      mailman_.setRollerIntake();
      pickup_front_.setPickupMode(PickupMode.PICKUP);
      shooter_.setRollerReverse();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter_.hasNote()){
          seen_note_ = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter_.setShootMode(ShootMode.IDLE);
    shooter_.rollerStop();
    mailman_.setRollerStop();
    pickup_front_.setPickupMode(PickupMode.IDLE);
    CommandScheduler.getInstance().schedule(new TeleRearPickup());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (seen_note_ == true && !shooter_.hasNote());
  }
}
