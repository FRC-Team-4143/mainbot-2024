package frc.robot.autos;

import frc.lib.AutoSequenceCommand;

public class SourceEFG extends AutoSequenceCommand {

    public SourceEFG(){
        setName("Source - EFG");
        addToNoteAuto("Source - E");
        addBranchAuto("E - Shoot - F", "E - F");
        addBranchAuto("F - Shoot - G", "F - G");
        addBranchWaitAuto("G - Shoot");
    }
    
}
