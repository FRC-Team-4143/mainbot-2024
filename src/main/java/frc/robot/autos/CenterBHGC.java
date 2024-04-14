package frc.robot.autos;

import frc.lib.AutoSequenceCommand;

public class CenterBHGC extends AutoSequenceCommand {

    public CenterBHGC(){
        setName("Center - BHGC");
        addToNoteAuto("Center - B moving - H");
        addBranchAuto("H - shoot - G", "H - G");
        // addToNoteAuto("G - shoot - C");
    }
    
}
