// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MailmanSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PickupSubsystem.PickupMode;

public class CleanAllPickups extends Command {
  /** Creates a new CleanAllPickups. */
  public CleanAllPickups() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PickupSubsystem.getMailmanInstance().setPickupMode(PickupMode.CLEAN);
    PickupSubsystem.getShooterInstance().setPickupMode(PickupMode.CLEAN);
    ShooterSubsystem.getInstance().setRollerReverse();
    MailmanSubsystem.getInstance().setRollerOutput();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PickupSubsystem.getMailmanInstance().setPickupMode(PickupMode.IDLE);
    PickupSubsystem.getShooterInstance().setPickupMode(PickupMode.IDLE);
    ShooterSubsystem.getInstance().rollerStop();
    MailmanSubsystem.getInstance().setRollerStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
