package frc.robot.autos;

import frc.lib.AutoSequenceCommand;
import frc.robot.AutoManager;

public class TestAuto extends AutoSequenceCommand {

    public TestAuto(){
        setName("Center - TEST");
        addToNoteAuto("Test Auto");
        AutoManager.getInstance().getAutoChooser().addOption(getName(), this);
    }
    
}
