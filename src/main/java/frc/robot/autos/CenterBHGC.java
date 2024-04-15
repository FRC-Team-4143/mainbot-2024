package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.lib.AutoSequenceCommand;
import frc.robot.commands.AutoShootAtSpeaker;

public class CenterBHGC extends AutoSequenceCommand {

    public CenterBHGC(){
        setName("Center - BHGC");
        addToNoteAuto("Center - B Moving - H");
        addBranchAuto("H - Shoot - G", "H - G");
        addToNoteAuto("G - Shoot - C");
        addToNoteAuto("Shoot Note");
    }
    
}
