package frc.robot.autos;

import frc.lib.AutoSequenceCommand;

public class CenterABC extends AutoSequenceCommand {

    public CenterABC(){
        setName("Center - BCAF");
        addToNoteAuto("Center - B Moving - C");
        addToNoteAuto("Shoot - C - A");
        addToNoteAuto("A - Shoot - F");
    }
    
}
