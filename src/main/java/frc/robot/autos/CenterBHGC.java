package frc.robot.autos;

import frc.lib.AutoSequenceCommand;

public class CenterBHGC extends AutoSequenceCommand {

    public CenterBHGC(){
        setName("Center - BHGC");
        addToNoteAuto("Center - B Moving - H");
        addBranchAuto("H - Shoot - G", "H - G");
        addToNoteAuto("G - Shoot - C");
        addToNoteAuto("Shoot Note");
    }
    
}
