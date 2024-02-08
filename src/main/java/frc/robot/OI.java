// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;
import frc.robot.subsystems.ShooterSubsystem.ShootTarget;

/** Add your docs here. */
public abstract class OI {

    //Sets up both controllers
    static CommandXboxController driver_joystick_ = new CommandXboxController(0); 
    // static CommandXboxController operator_joystick_ = new CommandXboxController(1);

    // ShooterSubsystem shooter_ = ShooterSubsystem.getInstance();
    // PickupSubsystem pickup_front_ = PickupSubsystem.getShooterInstance();

    public static void configureBindings(){

        SmartDashboard.putNumber("Shooter Speed", 0.75);
        SmartDashboard.putNumber("Wrist Speed", 0.1);

        SmartDashboard.putData("Set Wheel Offsets", Commands.runOnce(() -> SwerveDrivetrain.getInstance().tareEverything()).ignoringDisable(true));
        SmartDashboard.putData("Seed Field Centric", Commands.runOnce(() -> SwerveDrivetrain.getInstance().seedFieldRelative()).ignoringDisable(true));

        driver_joystick_.rightTrigger(0.5).whileTrue(Commands.startEnd(
            () -> {ShooterSubsystem.getInstance().setFlyWheelSpeed(SmartDashboard.getNumber("Shooter Speed", 0.75));
                    ShooterSubsystem.getInstance().setTarget(ShootTarget.SPEAKER); 
                    //SwerveDrivetrain.getInstance().setDriveMode(SwerveDrivetrain.DriveMode.TARGET);
                    ShooterSubsystem.getInstance().setShootMode(ShootMode.ACTIVETARGETING);}, 
            () -> {ShooterSubsystem.getInstance().flyWheelStop();
                    SwerveDrivetrain.getInstance().setDriveMode(SwerveDrivetrain.DriveMode.FIELD_CENTRIC);
                    ShooterSubsystem.getInstance().setShootMode(ShootMode.IDLE);}));

        // TODO: This Command does not use correct ShooterSubsystem Interfacing
        // THIS IS ONLY FOR PROTOTYPE TESTING!!!!
        driver_joystick_.rightBumper().whileTrue(Commands.startEnd(
            () -> {ShooterSubsystem.getInstance().setRollerFeed();
                    PickupSubsystem.getShooterInstance().setPickupMode();}, 
            () -> {ShooterSubsystem.getInstance().rollerStop();
                    PickupSubsystem.getShooterInstance().setIdleMode();}));

        // Backfeed Shooter Feeder
        driver_joystick_.leftBumper().whileTrue(Commands.startEnd(
            () -> ShooterSubsystem.getInstance().setRollerReverse(), 
            () -> ShooterSubsystem.getInstance().rollerStop()));

        // Wrist CCW
        driver_joystick_.x().whileTrue(Commands.startEnd(
            () -> ShooterSubsystem.getInstance().setWristSpeed(-SmartDashboard.getNumber("Wrist Speed", 0.1)), 
            () -> ShooterSubsystem.getInstance().wristStop(), 
            ShooterSubsystem.getInstance()));

        // Writst CW
        driver_joystick_.b().whileTrue(Commands.startEnd(
            () -> ShooterSubsystem.getInstance().setWristSpeed(SmartDashboard.getNumber("Wrist Speed", 0.1)), 
            () -> ShooterSubsystem.getInstance().wristStop(), 
            ShooterSubsystem.getInstance()));
            
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
    }


    static public double getDriverJoystickLeftX() {
        return driver_joystick_.getLeftX();
    }

    static public double getDriverJoystickLeftY() {
        return driver_joystick_.getLeftY();
    }

    static public double getDriverJoystickRightX() {
        return driver_joystick_.getRightX();
    }










}
