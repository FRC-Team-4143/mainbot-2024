package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import com.pathplanner.lib.auto.NamedCommands;

import frc.lib.AutoSequenceCommand;
import frc.robot.autos.SomeAuto;
import frc.robot.autos.TestAuto;
import frc.robot.commands.*;



public class AutoManager {

    // Singleton pattern
    private static AutoManager autoManagerInstance = null;

    public static AutoManager getInstance() {
        if (autoManagerInstance == null) {
            autoManagerInstance = new AutoManager();
        }
        return autoManagerInstance;
    }

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    private AutoManager() {
        NamedCommands.registerCommand("AutoShootAtSpeaker", new AutoShootAtSpeaker());//.withTimeout(2));
        NamedCommands.registerCommand("AutoShootAtSpeakerPreload", new AutoShootAtSpeakerPreload().withTimeout(2));
        NamedCommands.registerCommand("AllowVision", new SetIgnoreVision(false));
        NamedCommands.registerCommand("IgnoreVision", new SetIgnoreVision(true));
        NamedCommands.registerCommand("UpdateNoteRangeFlag", LimeLightSubsystem.getInstance().updateNoteRangeFlag());
        //NamedCommands.registerCommand("AutoFrontPickupToShooter", new HandoffToShooter().withTimeout(2).andThen(new AutoEnableDefaults()));
        
        SwerveDrivetrain.getInstance().configurePathPlanner();

        // Register each of the autos
        //registerAuto(new SomeAuto());
        registerAuto(new TestAuto());

        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public SendableChooser<Command> getAutoChooser(){
        return autoChooser;
    }

    public void registerAuto(AutoSequenceCommand auto){
        autoChooser.addOption(auto.getName(), auto);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
