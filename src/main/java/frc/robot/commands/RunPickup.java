// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PickupSubsystem.PickupMode;

public class RunPickup extends Command {
  /** Creates a new RunPickup. */
  public RunPickup() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PickupSubsystem.getShooterInstance().setPickupMode(PickupMode.PICKUP);

    if (!ShooterSubsystem.getInstance().hasNote()) {
      ShooterSubsystem.getInstance().setRollerFeed();
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PickupSubsystem.getShooterInstance().setPickupMode(PickupMode.IDLE);
    ShooterSubsystem.getInstance().rollerStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ShooterSubsystem.getInstance().hasNote();
  }
}
