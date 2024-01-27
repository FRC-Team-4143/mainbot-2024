package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;

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
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto mode", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
