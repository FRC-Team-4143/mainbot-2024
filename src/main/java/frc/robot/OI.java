// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.subsystems.MailmanSubsystem.HeightTarget;
import frc.robot.subsystems.PickupSubsystem.PickupMode;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;
import frc.robot.commands.*;

public abstract class OI {

    // Sets up both controllers
    static CommandXboxController driver_joystick_ = new CommandXboxController(0);
    static CommandXboxController operator_joystick_ = new CommandXboxController(1);

    static ShooterSubsystem shooter_ = ShooterSubsystem.getInstance();
    static PickupSubsystem pickup_front_ = PickupSubsystem.getMailmanInstance();
    static PickupSubsystem pickup_rear_ = PickupSubsystem.getShooterInstance();
    static MailmanSubsystem mailman_ = MailmanSubsystem.getInstance();
    static SwerveDrivetrain swerve_drivetrain_ = SwerveDrivetrain.getInstance();
    static ClimberSubsystem climber_ = ClimberSubsystem.getInstance();
    static Trigger crawlTrigger;

    public static void configureBindings() {

        SmartDashboard.putData("Set Wheel Offsets", Commands.runOnce(
                () -> swerve_drivetrain_.tareEverything())
                .ignoringDisable(true));
        SmartDashboard.putData("Seed Field Centric", Commands.runOnce(
                () -> swerve_drivetrain_.seedFieldRelative())
                .ignoringDisable(true));

        // Enagage Targeting
        driver_joystick_.rightTrigger(0.5).whileTrue(new ShootAtSpeaker());

        // Deliver the Mail
        driver_joystick_.leftTrigger(0.5).whileTrue(new ScoreMailman());

        // Rear Pickup
        driver_joystick_.rightBumper().whileTrue(new TeleRearPickup());

        // Front Pickup
        driver_joystick_.leftBumper().whileTrue(Commands.startEnd(
                () -> {
                        pickup_front_.setPickupMode(PickupMode.PICKUP);
                        mailman_.setRollerIntake();
                },
                () -> {
                        pickup_front_.setPickupMode(PickupMode.IDLE);
                        mailman_.setRollerStop();
                }));

        // Crawl
        crawlTrigger = new Trigger(() -> driver_joystick_.getHID().getPOV() > -1);
        crawlTrigger.whileTrue(new RobotCentricCrawl());

        // Mailman Rollers Out
        operator_joystick_.b().whileTrue(Commands.startEnd(
                () -> mailman_.setRollerOutput(),
                () -> mailman_.setRollerStop()));

        // Mailman Rollers In
        operator_joystick_.x().whileTrue(Commands.startEnd(
                () -> mailman_.setRollerIntake(),
                () -> mailman_.setRollerStop()));

        // Set Elevator to Amp Target
        operator_joystick_.y().whileTrue(Commands.runOnce(
                () -> mailman_.setHeight(HeightTarget.AMP)));

        // Set Elevator to Home Target
        operator_joystick_.a().whileTrue(Commands.runOnce(
                () -> mailman_.setHeight(HeightTarget.HOME)));

        // Handoff from Shooter to Mailman
        operator_joystick_.rightBumper().whileTrue(new HandoffToMailman());

        // Handoff from Mailman to Shooter
        operator_joystick_.leftBumper().whileTrue(new HandoffToShooter());


        // FOR PRACTICE MODE ONLY
        // Feed Shooter
        operator_joystick_.leftStick().whileTrue(Commands.startEnd(
                () -> {
                    shooter_.setRollerFeed();
                    pickup_rear_.setPickupMode(PickupMode.PICKUP);
                },
                () -> {
                    shooter_.rollerStop();
                    pickup_rear_.setPickupMode(PickupMode.IDLE);
                }));

        // Climb
        operator_joystick_.leftTrigger(0.1).whileTrue(Commands.startEnd(
                () -> climber_.setClimbSpeed(-0.6 * operator_joystick_.getLeftTriggerAxis()),
                () -> climber_.stopClimb()));

        // Reverse Climb
        operator_joystick_.rightTrigger(0.1).whileTrue(Commands.startEnd(
                () -> climber_.setClimbSpeed(0.6 * operator_joystick_.getRightTriggerAxis()),
                () -> climber_.stopClimb()));

        // Set Wrsit to Hook Position
        operator_joystick_.start().whileTrue(Commands.runOnce(
                () -> shooter_.setShootMode(ShootMode.CLIMB)));

        // Set Elevator to Trap Target
        operator_joystick_.back().whileTrue(Commands.runOnce(
                () -> mailman_.setHeight(HeightTarget.TRAP)));

        // Test buttons

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

    static public double getDriverJoystickPOVangle() {
        return driver_joystick_.getHID().getPOV();
    }
}
