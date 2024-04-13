package frc.robot.autos;

import frc.lib.AutoSequenceCommand;

public class SourceDEF extends AutoSequenceCommand {

    public SourceDEF(){
        setName("Source - DEF");
        addToNoteAuto("Source - D");
        addBranchAuto("D - Shoot - E", "D - E");
        addBranchAuto("E - Shoot - F", "E - F");
        addBranchWaitAuto("F - Shoot");
    }
    
}
