package frc.robot;

import java.lang.reflect.Field;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

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
        NamedCommands.registerCommand("AutoEnableDefaults", new AutoEnableDefaults());
        NamedCommands.registerCommand("AutoShootAtSpeaker", new AutoShootAtSpeaker().withTimeout(2));
        NamedCommands.registerCommand("PPShootAtSpeaker", new PPShootAtSpeaker().withTimeout(5));
        NamedCommands.registerCommand("Duck", new Duck());
        SwerveDrivetrain.getInstance().configurePathPlanner();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
