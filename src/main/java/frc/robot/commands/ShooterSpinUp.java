// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;
import frc.robot.subsystems.SwerveDrivetrain;

public class ShooterSpinUp extends Command {
  /** Creates a new ShootAtTarget. */
  boolean shot_note_;

  public ShooterSpinUp() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterSubsystem.getInstance().setShootMode(ShootMode.SPINUP);
    SwerveDrivetrain.getInstance().setDriveMode(SwerveDrivetrain.DriveMode.SPINUP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSubsystem.getInstance().setShootMode(ShootMode.IDLE);
    SwerveDrivetrain.getInstance().setDriveMode(SwerveDrivetrain.DriveMode.FIELD_CENTRIC);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
