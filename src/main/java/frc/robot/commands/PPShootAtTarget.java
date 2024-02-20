// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;
import frc.robot.subsystems.ShooterSubsystem.ShootTarget;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.PickupSubsystem.PickupMode;

public class PPShootAtTarget extends Command {
  /** Creates a new PPShootAtTarget. */
  ShootTarget target;
  public PPShootAtTarget(ShootTarget targetRequest) {
    target = targetRequest;
    addRequirements(ShooterSubsystem.getInstance());
    addRequirements(PickupSubsystem.getShooterInstance());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterSubsystem.getInstance().setFlyWheelSpeed(SmartDashboard.getNumber("Shooter Speed", 0.75));
    ShooterSubsystem.getInstance().setTarget(target);
    SwerveDrivetrain.getInstance().setDriveMode(SwerveDrivetrain.DriveMode.AUTONOMOUS_TARGET);
    ShooterSubsystem.getInstance().setShootMode(ShootMode.TARGET);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ShooterSubsystem.getInstance().isTargetLocked()){
      ShooterSubsystem.getInstance().setRollerFeed();
      PickupSubsystem.getShooterInstance().setPickupMode(PickupMode.PICKUP);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSubsystem.getInstance().flyWheelStop();
    SwerveDrivetrain.getInstance().setDriveMode(SwerveDrivetrain.DriveMode.AUTONOMOUS);
    ShooterSubsystem.getInstance().setShootMode(ShootMode.IDLE);
    ShooterSubsystem.getInstance().rollerStop();
    PickupSubsystem.getShooterInstance().setPickupMode(PickupMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
