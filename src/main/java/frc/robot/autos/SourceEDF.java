package frc.robot.autos;

import frc.lib.AutoSequenceCommand;

public class SourceEDF extends AutoSequenceCommand {

    public SourceEDF(){
        setName("Source - EDF");
        addToNoteAuto("Source - E");
        addBranchAuto("E - Shoot - D", "E - D");
        addBranchAuto("D - Shoot - F", "D - F");
        addBranchWaitAuto("F - Shoot");
    }
    
}
