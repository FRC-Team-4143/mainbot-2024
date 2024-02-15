package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem.ShootTarget;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.*;


public class AutoManager {

    // Singleton pattern
    private static AutoManager exampleInstance = null;

    public static AutoManager getInstance() {
        if (exampleInstance == null) {
            exampleInstance = new AutoManager();
        }
        return exampleInstance;
    }

    private final SendableChooser<Command> autoChooser;

    private AutoManager() {
        NamedCommands.registerCommand("ShootAtSpeaker", new PPShootAtTarget(ShootTarget.SPEAKER));
        NamedCommands.registerCommand("RunPickup", new RunPickup());
        NamedCommands.registerCommand("LaunchNote", new LaunchNote());
        SwerveDrivetrain.getInstance().configurePathPlanner();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
