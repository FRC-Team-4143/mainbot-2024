// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.MailmanSubsystem.HeightTarget;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import frc.robot.subsystems.MailmanSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;

public class ScoreMailman extends Command {
  /** Creates a new ScoreMailman. */
  public ScoreMailman() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveDrivetrain.getInstance().setDriveMode(DriveMode.TARGET);
    if(ShooterSubsystem.getInstance().hasNote()){
      CommandScheduler.getInstance().schedule(new HandoffToMailman().withTimeout(1).andThen(() -> MailmanSubsystem.getInstance().setHeight(HeightTarget.AMP)));
    } else {
      MailmanSubsystem.getInstance().setHeight(HeightTarget.AMP);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    MailmanSubsystem.getInstance().setTargetYaw();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().schedule(Commands.run(
      () -> MailmanSubsystem.getInstance().setRollerOutput()).withTimeout(0.5)
      .andThen(() -> {
        MailmanSubsystem.getInstance().setHeight(HeightTarget.HOME);
        MailmanSubsystem.getInstance().setRollerStop();
      }));
    SwerveDrivetrain.getInstance().setDriveMode(DriveMode.FIELD_CENTRIC);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
