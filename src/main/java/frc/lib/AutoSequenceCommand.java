// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FetchNote;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;

public abstract class AutoSequenceCommand extends SequentialCommandGroup {

    BooleanSupplier isNoteAvaibale = () -> LimeLightSubsystem.getInstance().isNoteAvaibale();
    BooleanSupplier isRobotHoldingNote = () -> 
        ShooterSubsystem.getInstance().hasNote() || 
        PickupSubsystem.getMailmanInstance().hasNote() || 
        PickupSubsystem.getShooterInstance().hasNote();

    /**
     * Adds a branch auto decision into the auto sequence
     * @param desiredAuto auto to follow if condition is true (go to shoot -> shoot -> next note)
     * @param alternateAuto auto to follow if condition is false (go to next note)
     */
    public void addBranchAuto(String desiredAuto, String alternateAuto){
        addCommands(createShootOrAlternateNoteAuto(desiredAuto, alternateAuto));
    }

    /**
     * Adds a single auto with note correction into the auto sequence
     * @param auto auto to follow
     */
    public void addToNoteAuto(String auto){
        addCommands(createToNoteAuto(auto));
    }

    /**
     * Adds a branch auto decision into the auto sequence
     * @param desiredAuto auto to follow if condition is true (go to shoot -> shoot -> next note)
     */
    public void addBranchWaitAuto(String desiredAuto){
        addCommands(createShootOrWaitAuto(desiredAuto));
    }

    /**
     * Creates a branch auto decision
     * @param desiredAuto auto to follow if condition is true (go to shoot -> shoot -> next note)
     * @param alternateAuto auto to follow if condition is false (go to next note)
     * @param cond condition to deterime which path is followed
     * @return branching auto command
     */
    private Command createShootOrAlternateNoteAuto(String desiredAuto, String alternateAuto){
        return Commands.either(
            createToNoteAuto(desiredAuto),
            createToNoteAuto(alternateAuto),
            isRobotHoldingNote);
    }

    /**
     * Creates a single auto with note correction
     * @param path path to follow
     * @return command to follow path with note correction
     */
    private Command createToNoteAuto(String auto){
        return new SwitchCommand(new PathPlannerAuto(auto), new FetchNote().withTimeout(2), isNoteAvaibale);
    }


    /**
     * Creates a branch auto decision with wait option
     * @param desiredAuto auto to follow if condition is true (go to shoot -> shoot -> next note)
     * @return branching auto command
     */
    private Command createShootOrWaitAuto(String desiredAuto){
        return Commands.either(
            new PathPlannerAuto(desiredAuto),
            new WaitCommand(15),
            isRobotHoldingNote);
    }
}
