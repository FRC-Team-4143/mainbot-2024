// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutoCommand {
    SequentialCommandGroup auto_command;

    BooleanSupplier isRobotHoldingNote = () -> 
        ShooterSubsystem.getInstance().hasNote() || 
        PickupSubsystem.getMailmanInstance().hasNote() || 
        PickupSubsystem.getShooterInstance().hasNote();

    public AutoCommand(String name){
        auto_command.setName(name);
    }

    /**
     * Adds a branch auto decision into the auto sequence
     * @param desiredAuto auto to follow if condition is true
     * @param backupAuto auto to follow if condition is false
     * @param cond condition to deterime which path is followed
     * @return updated sequential command group
     */
    public AutoCommand addShootOrBackupAuto(String desiredAuto, String backupAuto){
        auto_command.addCommands(Commands.either(
            new PathPlannerAuto(desiredAuto),
            createToNotePath(backupAuto),
            isRobotHoldingNote));
        return this;
    }

    /**
     * Adds a single auto with note correction into the auto sequence
     * @param auto auto to follow
     * @return updated sequential command group
     */
    public AutoCommand addToNoteAuto(String auto){
        auto_command.addCommands(createToNotePath(auto));
        return this;
    }

    /**
     * Creates a single auto with note correction
     * @param path path to follow
     * @return command to follow path with note correction
     */
    private Command createToNotePath(String auto){
        return new PathPlannerAuto(auto)
            .until(null)
            .withInterruptBehavior(null);
    }

    /**
     * Adds a simple action into the auto sequence
     * @param action command to execute
     * @return updated sequential command group
     */
    public AutoCommand addAction(Command action){
        auto_command.addCommands(action);
        return this;
    }

    /**
     * Returns the completed auto command
     * @return auto sequential command group
     */
    public SequentialCommandGroup getAutoCommand(){
        return auto_command;
    }

    public String getAutoCommandName(){
        return auto_command.getName();
    }
}
