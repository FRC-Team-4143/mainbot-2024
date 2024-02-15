// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;
import frc.robot.subsystems.ShooterSubsystem.ShootTarget;
import frc.robot.commands.*;

/** Add your docs here. */
public abstract class OI {

    // Sets up both controllers
    static CommandXboxController driver_joystick_ = new CommandXboxController(0);
    static CommandXboxController operator_joystick_ = new CommandXboxController(1);

    public static void configureBindings() {

        SmartDashboard.putData("Set Wheel Offsets",
                Commands.runOnce(() -> SwerveDrivetrain.getInstance().tareEverything())
                        .ignoringDisable(true));
        SmartDashboard.putData("Seed Field Centric",
                Commands.runOnce(() -> SwerveDrivetrain.getInstance().seedFieldRelative())
                        .ignoringDisable(true));

        driver_joystick_.rightTrigger(0.5).whileTrue(new ShootAtTarget(ShootTarget.SPEAKER));

        // TODO: This Command does not use correct ShooterSubsystem Interfacing
        // THIS IS ONLY FOR PROTOTYPE TESTING!!!!
        driver_joystick_.rightBumper().whileTrue(Commands.startEnd(
                () -> {
                    ShooterSubsystem.getInstance().setRollerFeed();
                    PickupSubsystem.getShooterInstance().setPickupMode();
                },
                () -> {
                    ShooterSubsystem.getInstance().rollerStop();
                    PickupSubsystem.getShooterInstance().setIdleMode();
                }));

        // Backfeed Shooter Feeder
        // driver_joystick_.leftBumper().whileTrue(Commands.startEnd(
        // () -> ShooterSubsystem.getInstance().setRollerReverse(),
        // () -> ShooterSubsystem.getInstance().rollerStop()));
        driver_joystick_.leftBumper().whileTrue(new ShooterBackfeed());

        // Wrist CCW
        // driver_joystick_.x().whileTrue(Commands.startEnd(
        // () ->
        // ShooterSubsystem.getInstance().setWristSpeed(-SmartDashboard.getNumber("Wrist
        // Speed", 0.1)),
        // () -> ShooterSubsystem.getInstance().wristStop(),
        // ShooterSubsystem.getInstance()));
        
        // // Writst CW
        // driver_joystick_.b().whileTrue(Commands.startEnd(
        // () ->
        // ShooterSubsystem.getInstance().setWristSpeed(SmartDashboard.getNumber("Wrist
        // Speed", 0.1)),
        // () -> ShooterSubsystem.getInstance().wristStop(),
        // ShooterSubsystem.getInstance()));


        // Run Pickup
        driver_joystick_.leftTrigger(0.5).whileTrue(Commands.startEnd(
                () -> PickupSubsystem.getShooterInstance().setPickupMode(),
                () -> PickupSubsystem.getShooterInstance().setIdleMode(),
                PickupSubsystem.getShooterInstance()));

        // Run Pickup Rev
        driver_joystick_.a().whileTrue(Commands.startEnd(
                () -> PickupSubsystem.getShooterInstance().setCleanMode(),
                () -> PickupSubsystem.getShooterInstance().setIdleMode(),
                PickupSubsystem.getShooterInstance()));

        driver_joystick_.start().whileTrue(Commands.startEnd(
                () -> PickupSubsystem.getMailmanInstance().setPickupMode(),
                () -> PickupSubsystem.getMailmanInstance().setIdleMode(),
                PickupSubsystem.getMailmanInstance()));

        driver_joystick_.y().whileTrue(Commands.startEnd(
                () -> ShooterSubsystem.getInstance().setFlyWheelSpeed(-0.1),
                () -> ShooterSubsystem.getInstance().flyWheelStop()));

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
