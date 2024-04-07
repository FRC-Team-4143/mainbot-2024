package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.autos.SomeAuto;
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

    private SendableChooser<Command> autoChooser;

    private SomeAuto someAuto = new SomeAuto();

    

    private AutoManager() {
        NamedCommands.registerCommand("AutoShootAtSpeaker", new AutoShootAtSpeaker());//.withTimeout(2));
        NamedCommands.registerCommand("AutoShootAtSpeakerPreload", new AutoShootAtSpeakerPreload().withTimeout(2));
        NamedCommands.registerCommand("AllowVision", new SetIgnoreVision(false));
        NamedCommands.registerCommand("IgnoreVision", new SetIgnoreVision(true));
        NamedCommands.registerCommand("UpdateNoteRangeFlag", LimeLightSubsystem.getInstance().updateNoteRangeFlag());
        //NamedCommands.registerCommand("AutoFrontPickupToShooter", new HandoffToShooter().withTimeout(2).andThen(new AutoEnableDefaults()));

        SwerveDrivetrain.getInstance().configurePathPlanner();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public SendableChooser<Command> getAutoChooser(){
        return autoChooser;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
