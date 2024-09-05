package frc.robot;

import com.choreo.lib.Choreo;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;

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
        autoChooser.addOption("Straight Back", SwerveDrivetrain.getInstance().followPathCommand(Choreo.getTrajectory("TestTraj")));
        autoChooser.addOption("Note H", SwerveDrivetrain.getInstance().followPathCommand(Choreo.getTrajectory("TestTraj")));
    }

    public SendableChooser<Command> getAutoChooser(){
        return autoChooser;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
