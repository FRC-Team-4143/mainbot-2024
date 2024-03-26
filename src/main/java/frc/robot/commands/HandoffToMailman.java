// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MailmanSubsystem;
import frc.robot.subsystems.MailmanSubsystem.HeightTarget;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.PickupSubsystem.PickupMode;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;

public class HandoffToMailman extends Command {
    static ShooterSubsystem shooter_ = ShooterSubsystem.getInstance();
    static PickupSubsystem pickup_front_ = PickupSubsystem.getMailmanInstance();
    static PickupSubsystem pickup_rear_ = PickupSubsystem.getShooterInstance();
    static MailmanSubsystem mailman_ = MailmanSubsystem.getInstance();
    private boolean has_seen_note_ = false;

    /** Creates a new HandoffToMailman. */
    public HandoffToMailman() {
        addRequirements(shooter_);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mailman_.setHeight(HeightTarget.HOME);
        shooter_.setShootMode(ShootMode.TRANSFER);
        has_seen_note_ = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (mailman_.atHeight() && shooter_.wristLocked()) {
            mailman_.setRollerRecieve();
            pickup_rear_.setPickupMode(PickupMode.PICKUP);
            // pickup_front_.setPickupMode(PickupMode.PICKUP);
            shooter_.setRollerFeed();
        } else {
            mailman_.setHeight(HeightTarget.HOME);
            shooter_.setShootMode(ShootMode.TRANSFER);
            mailman_.setRollerStop();
            pickup_rear_.setPickupMode(PickupMode.IDLE);
            shooter_.rollerStop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter_.setShootMode(ShootMode.IDLE);
        shooter_.rollerStop();
        mailman_.setRollerStop();
        pickup_rear_.setPickupMode(PickupMode.IDLE);
        has_seen_note_ = has_seen_note_ | pickup_front_.hasNote();
    }

    // Returns true when the command should end.y
    @Override
    public boolean isFinished() {
        return has_seen_note_ && !pickup_front_.hasNote();
    }
}
