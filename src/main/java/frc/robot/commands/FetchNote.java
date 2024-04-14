// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;

public class FetchNote extends Command {
  /** Creates a new FetchNote. */
  public FetchNote() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SwerveDrivetrain.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveDrivetrain.getInstance().setDriveMode(DriveMode.NOTE_TARGET);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveDrivetrain.getInstance().applyChassisSpeeds(new ChassisSpeeds());
    SwerveDrivetrain.getInstance().setDriveMode(DriveMode.AUTONOMOUS);
    LimeLightSubsystem.getInstance().resetNoteRangeFlag();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PickupSubsystem.getShooterInstance().hasNote();
  }
}
