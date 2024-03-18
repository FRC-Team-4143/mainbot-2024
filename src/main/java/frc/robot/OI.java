// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.subsystems.MailmanSubsystem.HeightTarget;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;
import frc.robot.subsystems.ShooterSubsystem.ShootTarget;
import frc.lib.Util;
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
                () -> swerve_drivetrain_.seedFieldRelative(swerve_drivetrain_.getDriverPrespective()))
                .ignoringDisable(true));
        SmartDashboard.putData("Reset Climber Encoder", Commands.runOnce(
                () -> ClimberSubsystem.getInstance().resetClimberEncoder())
                .ignoringDisable(true));
        SmartDashboard.putData("Pause Vision", Commands.runOnce(
                () -> PoseEstimator.getInstance().pauseVisionFilter())
                .ignoringDisable(true));

        BooleanSupplier isTestMode = () -> DriverStation.isTest();
        BooleanSupplier isTargetModeSpeaker = () -> shooter_.getShootTarget() == ShootTarget.SPEAKER;
        BooleanSupplier isRobotHoldingNote = () -> shooter_.hasNote() || pickup_front_.hasNote() || pickup_rear_.hasNote();
        BooleanSupplier isRearIntakeStagingNote = () -> pickup_rear_.hasNote() && !shooter_.hasNote() && !pickup_front_.hasNote();

        // ------------------        
        // Driver Controls
        // ------------------

        // Shoot at Speaker or Pass
        driver_joystick_.rightTrigger(0.5).whileTrue(new ConditionalCommand(new TeleShootAtSpeaker(), new TelePass(), isTargetModeSpeaker));

        // Deliver the Mail
        driver_joystick_.leftTrigger(0.5).whileTrue(new ScoreMailman());

        // Rear Pickup
        driver_joystick_.rightBumper().whileTrue(new TeleRearPickup().unless(isRobotHoldingNote));
        driver_joystick_.rightBumper().onFalse(new TeleRearPickupIndex().withTimeout(5).onlyIf(isRearIntakeStagingNote));
        
        // Front Pickup
        driver_joystick_.leftBumper().whileTrue(new TeleFrontPickup().unless(isRobotHoldingNote));
        //driver_joystick_.leftBumper().onFalse(new TeleFrontPickupIndex());

        // Crawl
        crawlTrigger = new Trigger(() -> driver_joystick_.getHID().getPOV() > -1);
        crawlTrigger.whileTrue(new RobotCentricCrawl());

        // ------------------        
        // Operator Controls
        // ------------------

        // Mailman Rollers Out
        operator_joystick_.b().whileTrue(Commands.startEnd(
                () -> mailman_.setRollerOutput(),
                () -> mailman_.setRollerStop()));

        // Mailman Rollers In
        operator_joystick_.x().whileTrue(Commands.startEnd(
                () -> mailman_.setRollerSpeed(0.25),
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

        // Manual Shoot
        operator_joystick_.rightTrigger(0.5).whileTrue(new ConditionalCommand(new OverrideShootAtSpeaker(), new OverrideTelePass(), isTargetModeSpeaker));

        // Spinup Shooter
        operator_joystick_.leftTrigger(0.5).whileTrue(new ShooterSpinUp());

        // Empty All Pickups
        operator_joystick_.leftStick().whileTrue(new CleanAllPickups());

         // Toggle Pass and Speaker Modes
        operator_joystick_.rightStick().onTrue(Commands.runOnce(
            () -> shooter_.toggleShootTarget()));

        // Endgame Climb step increment
        operator_joystick_.start().whileTrue(Commands.runOnce(() -> climber_.scheduleNextEndgameState()));

        // Endgame Climb step decrement
        operator_joystick_.back().whileTrue(Commands.runOnce(() -> climber_.schedulePreviousEndgameState()));

        // Climb
        operator_joystick_.povUp().whileTrue(Commands.startEnd(
                () -> climber_.setClimbSpeed(-0.6),
                () -> climber_.stopClimb()));
            
        operator_joystick_.povDown().whileTrue(new ManuallyLowerElevator());

        // Test buttons
        driver_joystick_.b().whileTrue(new SwerveProfile(4, 0, 0).onlyIf(isTestMode));

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

    static public boolean getDriverJoystickRightY() {
        double val = driver_joystick_.getRightY();
        return Util.epislonEquals(val, 0, 0.1);
    }

    static public double getDriverJoystickPOVangle() {
        return driver_joystick_.getHID().getPOV();
    }

    static public boolean getDriverRightTriggerPulled(){
        return driver_joystick_.getRightTriggerAxis() > 0.1;
    }

    static public boolean getOperatorLeftTriggerPulled(){
        return operator_joystick_.getLeftTriggerAxis() > 0.1;
    }
}
