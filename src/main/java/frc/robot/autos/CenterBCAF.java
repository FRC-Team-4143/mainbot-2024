package frc.robot.autos;

import frc.lib.AutoSequenceCommand;

public class CenterBCAF extends AutoSequenceCommand {

    public CenterBCAF(){
        setName("Center - BCAF");
        addToNoteAuto("Center - B Moving - C");
        addToNoteAuto("Shoot - C - A");
        addToNoteAuto("A - Shoot - F");
    }
    
}
