// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MailmanSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.PickupSubsystem.PickupMode;

public class TeleFrontPickup extends Command {
  public TeleFrontPickup() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PickupSubsystem.getMailmanInstance().setPickupMode(PickupMode.PICKUP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (!MailmanSubsystem.getInstance().hasNote()) {
      MailmanSubsystem.getInstance().setRollerIntake();
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PickupSubsystem.getMailmanInstance().setPickupMode(PickupMode.IDLE);
    MailmanSubsystem.getInstance().setRollerStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//MailmanSubsystem.getInstance().hasNote();
  }
}
