// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.subsystems.MailmanSubsystem.HeightTarget;
import frc.robot.subsystems.PickupSubsystem.PickupMode;
import frc.robot.subsystems.ShooterSubsystem.ShootTarget;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
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
        BooleanSupplier isAutomaticShotMode = () -> shooter_.isAutomaticAimMode();
        BooleanSupplier isRobotHoldingNote = () -> shooter_.hasNote() || pickup_front_.hasNote()
                || pickup_rear_.hasNote();
        BooleanSupplier isRearIntakeStagingNote = () -> pickup_rear_.hasNote() && !shooter_.hasNote()
                && !pickup_front_.hasNote();
        BooleanSupplier isFrontIntakeStagingNote = () -> pickup_front_.hasNote() && !shooter_.hasNote();
        BooleanSupplier isMailmanReady = () -> (mailman_.getTarget() != HeightTarget.HOME);
        BooleanSupplier isHandingOff = () -> (shooter_.isShooterHandoffState());
        BooleanSupplier isClimbing = () -> (climber_.getEndgameState() >= 2);

        // ------------------
        // Driver Controls
        // ------------------

        // Shoot at Speaker or Pass
        driver_joystick_.rightTrigger(0.5).whileTrue(
                new ConditionalCommand(
                        new ScoreMailman(),
                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new TeleShootAtSpeaker().unless(isMailmanReady),
                                        new TelePass().unless(isFrontIntakeStagingNote),
                                        isTargetModeSpeaker),
                                new ConditionalCommand(
                                        new OverrideShootAtSpeaker(),
                                        new OverrideTelePass(),
                                        isTargetModeSpeaker),
                                isAutomaticShotMode),
                        isMailmanReady));

        // Move the Elevator to Amp Position
        driver_joystick_.leftTrigger(0.5).whileTrue(Commands.startEnd(
                () -> {
                    mailman_.setHeight(HeightTarget.AMP);
                    swerve_drivetrain_.setTargetRotation(
                            swerve_drivetrain_.getDriverPrespective().rotateBy(Rotation2d.fromDegrees(90)));
                    swerve_drivetrain_.setDriveMode(DriveMode.TARGET);
                },
                () -> {
                    mailman_.setHeight(HeightTarget.HOME);
                    swerve_drivetrain_.setDriveMode(DriveMode.FIELD_CENTRIC);
                }).unless(isHandingOff));

        // Rear Pickup
        driver_joystick_.rightBumper().whileTrue(new TeleRearPickup().unless(isRobotHoldingNote).ignoringDisable(true));
        driver_joystick_.rightBumper()
                .onFalse(new TeleRearPickupIndex().withTimeout(5).onlyIf(isRearIntakeStagingNote));

        // Front Pickup
        driver_joystick_.leftBumper()
                .whileTrue(new TeleFrontPickup().unless(isRobotHoldingNote).withTimeout(2).ignoringDisable(true));
        // driver_joystick_.leftBumper().onFalse(new
        // TeleFrontPickupIndex().withTimeout(5).onlyIf(isFrontIntakeStagingNote));

        // Crawl
        crawlTrigger = new Trigger(() -> driver_joystick_.getHID().getPOV() > -1);
        crawlTrigger.whileTrue(new RobotCentricCrawl());

        // Bump Climber Setpoint Down
        driver_joystick_.a().onTrue(Commands.runOnce(
                () -> climber_.lowerHeightTarget()).onlyIf(isClimbing));

        // ------------------
        // Operator Controls
        // ------------------

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

        // Handoff from Mailman to Shooter
        operator_joystick_.leftBumper().onTrue(new HandoffToShooter().withTimeout(2.0));

        // Handoff from Shooter to Mailman
        operator_joystick_.rightBumper().onTrue(new HandoffToMailman().withTimeout(2.0));

        // Spinup Shooter
        operator_joystick_.leftTrigger(0.5).whileTrue(new ShooterSpinUp().unless(isFrontIntakeStagingNote));

        // Empty All Pickups
        operator_joystick_.rightTrigger(0.5).whileTrue(new CleanAllPickups());

        // Toggle Pass and Speaker Modes
        operator_joystick_.leftStick().onTrue(Commands.runOnce(
                () -> shooter_.toggleShootTarget()));

        // Toggle Manual vs Automatic
        operator_joystick_.rightStick().onTrue(Commands.runOnce(
                () -> shooter_.toggleAutomaticAimMode()));

        // Endgame Climb step increment
        operator_joystick_.start()
                .whileTrue(Commands.runOnce(() -> climber_.scheduleNextEndgameState()).unless(isHandingOff));

        // Endgame Climb step decrement
        operator_joystick_.back()
                .whileTrue(Commands.runOnce(() -> climber_.schedulePreviousEndgameState()).unless(isHandingOff));

        // Climb
        operator_joystick_.povUp().whileTrue(Commands.startEnd(
                () -> climber_.setClimbSpeed(0.6),
                () -> climber_.stopClimb()));

        operator_joystick_.povDown().whileTrue(new ManuallyLowerElevator());

        // -------------
        // Test buttons
        // -------------
        driver_joystick_.b().whileTrue(new SwerveProfile(4, 0, 0).onlyIf(isTestMode));

        driver_joystick_.x().whileTrue(Commands.startEnd(
                () -> {
                    swerve_drivetrain_.setDriveMode(DriveMode.NOTE_TARGET);
                    pickup_rear_.setPickupMode(PickupMode.PICKUP);
                },
                () -> {
                    swerve_drivetrain_.setDriveMode(DriveMode.FIELD_CENTRIC);
                    pickup_rear_.setPickupMode(PickupMode.IDLE);
                    CommandScheduler.getInstance().schedule(new TeleRearPickupIndex().withTimeout(2));
                }).onlyIf(isTestMode));

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

    static public boolean getDriverRightTriggerPulled() {
        return driver_joystick_.getRightTriggerAxis() > 0.1;
    }

    static public boolean getOperatorLeftTriggerPulled() {
        return operator_joystick_.getLeftTriggerAxis() > 0.1;
    }
}
