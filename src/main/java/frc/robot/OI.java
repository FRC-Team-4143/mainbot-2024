// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.MailmanSubsystem.HeightTarget;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;
import frc.robot.subsystems.ShooterSubsystem.ShootTarget;

/** Add your docs here. */
public abstract class OI {

    // Sets up both controllers
    static CommandXboxController driver_joystick_ = new CommandXboxController(0);
    static CommandXboxController operator_joystick_ = new CommandXboxController(1);
    // static CommandXboxController operator_joystick_ = new
    // CommandXboxController(1);

    // ShooterSubsystem shooter_ = ShooterSubsystem.getInstance();
    // PickupSubsystem pickup_front_ = PickupSubsystem.getShooterInstance();

    public static void configureBindings() {

        SmartDashboard.putData("Set Wheel Offsets",
                Commands.runOnce(() -> SwerveDrivetrain.getInstance().tareEverything())
                        .ignoringDisable(true));
        SmartDashboard.putData("Seed Field Centric",
                Commands.runOnce(() -> SwerveDrivetrain.getInstance().seedFieldRelative())
                        .ignoringDisable(true));

        // Enagage Targeting
        driver_joystick_.rightTrigger(0.5).whileTrue(Commands.startEnd(
                () -> {
                    ShooterSubsystem.getInstance().setFlyWheelSpeed(0.75);
                    ShooterSubsystem.getInstance().setTarget(ShootTarget.SPEAKER);
                    SwerveDrivetrain.getInstance().setDriveMode(SwerveDrivetrain.DriveMode.TARGET);
                    ShooterSubsystem.getInstance().setShootMode(ShootMode.ACTIVETARGETING);
                },
                () -> {
                    ShooterSubsystem.getInstance().flyWheelStop();
                    SwerveDrivetrain.getInstance()
                            .setDriveMode(SwerveDrivetrain.DriveMode.FIELD_CENTRIC);
                    ShooterSubsystem.getInstance().setShootMode(ShootMode.IDLE);
                }));

        // Deliver the Mail
        driver_joystick_.leftTrigger(0.5).whileTrue(Commands.startEnd(
                () -> MailmanSubsystem.getInstance().setRollerOutput(),
                () -> MailmanSubsystem.getInstance().setRollerStop()));

        // Rear Pickup
        driver_joystick_.rightBumper().whileTrue(Commands.startEnd(
                () -> PickupSubsystem.getShooterInstance().setPickupMode(),
                () -> PickupSubsystem.getShooterInstance().setIdleMode(),
                PickupSubsystem.getShooterInstance()));

        // Front Pickup
        driver_joystick_.leftBumper().whileTrue(Commands.startEnd(
                () -> PickupSubsystem.getMailmanInstance().setPickupMode(),
                () -> PickupSubsystem.getMailmanInstance().setIdleMode(),
                PickupSubsystem.getShooterInstance()));


        // Mailman Rollers Out
        operator_joystick_.b().whileTrue(Commands.startEnd(
                () -> MailmanSubsystem.getInstance().setRollerOutput(),
                () -> MailmanSubsystem.getInstance().setRollerStop()));

        // Mailman Rollers In
        operator_joystick_.x().whileTrue(Commands.startEnd(
                () -> MailmanSubsystem.getInstance().setRollerIntake(),
                () -> MailmanSubsystem.getInstance().setRollerStop()));

        // Set Elevator to Amp Target
        operator_joystick_.y().whileTrue(Commands.runOnce(
                () -> MailmanSubsystem.getInstance().setHeight(HeightTarget.AMP)));

        // Set Elevator to Home Target
        operator_joystick_.a().whileTrue(Commands.runOnce(
            () -> MailmanSubsystem.getInstance().setHeight(HeightTarget.HOME)));

        // Handoff from shooter to Mailman
        operator_joystick_.rightBumper().whileTrue(Commands.startEnd(
            () -> {
                MailmanSubsystem.getInstance().setHeight(HeightTarget.HOME);
                ShooterSubsystem.getInstance().setShootMode(ShootMode.TRANSFER);
                MailmanSubsystem.getInstance().setRollerOutput();
                PickupSubsystem.getShooterInstance().setPickupMode();
                ShooterSubsystem.getInstance().setRollerFeed();
            },
            () -> {
                ShooterSubsystem.getInstance().setShootMode(ShootMode.IDLE);
                ShooterSubsystem.getInstance().rollerStop();
                MailmanSubsystem.getInstance().setRollerStop();
                PickupSubsystem.getShooterInstance().setIdleMode();
            }));

        // FOR PRACTICE MODE ONLY
        // Load Shooter
        operator_joystick_.leftBumper().whileTrue(Commands.startEnd(
            () -> {
                ShooterSubsystem.getInstance().setRollerFeed();
                PickupSubsystem.getShooterInstance().setPickupMode();
            },
            () -> {
                ShooterSubsystem.getInstance().rollerStop();
                PickupSubsystem.getShooterInstance().setIdleMode();
            }));

        operator_joystick_.leftTrigger(0.1).whileTrue(Commands.startEnd(
            () -> ClimberSubsystem.getInstance().setClimbSpeed(-0.4 * operator_joystick_.getLeftTriggerAxis()),
            () -> ClimberSubsystem.getInstance().stopClimb()));

        operator_joystick_.rightTrigger(0.1).whileTrue(Commands.startEnd(
            () -> ClimberSubsystem.getInstance().setClimbSpeed(0.4 * operator_joystick_.getRightTriggerAxis()),
            () -> ClimberSubsystem.getInstance().stopClimb()));

        operator_joystick_.start().whileTrue(Commands.runOnce(
            () -> ShooterSubsystem.getInstance().setShootMode(ShootMode.CLIMB)));

        // Set Elevator to Amp Target
        operator_joystick_.back().whileTrue(Commands.runOnce(
            () -> MailmanSubsystem.getInstance().setHeight(HeightTarget.TRAP)));
    }

    static public double getDriverJoystickLeftX() {
        double val = driver_joystick_.getLeftX();
        double output = val * val;
        output = Math.copySign(output, val);
        return output;
    }

    static public double getDriverJoystickLeftY() {
        double val = driver_joystick_.getLeftY();
        double output = val * val;
        output = Math.copySign(output, val);
        return output;
    }

    static public double getDriverJoystickRightX() {
        double val = driver_joystick_.getRightX();
        double output = val * val;
        output = Math.copySign(output, val);
        return output;
    }
}
