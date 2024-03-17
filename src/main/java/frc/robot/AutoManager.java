package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;
import com.pathplanner.lib.auto.NamedCommands;

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

    private final SendableChooser<Command> autoChooser;

    private AutoManager() {
        NamedCommands.registerCommand("AutoShootAtSpeaker", new AutoShootAtSpeaker());//.withTimeout(2));
        NamedCommands.registerCommand("AutoShootAtSpeakerPreload", new AutoShootAtSpeakerPreload().withTimeout(2));
        NamedCommands.registerCommand("Duck", new Duck());
        NamedCommands.registerCommand("AllowVision", new SetIgnoreVision(false));
        NamedCommands.registerCommand("IgnoreVision", new SetIgnoreVision(true));
        //NamedCommands.registerCommand("AutoFrontPickupToShooter", new HandoffToShooter().withTimeout(2).andThen(new AutoEnableDefaults()));
        SwerveDrivetrain.getInstance().configurePathPlanner();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
