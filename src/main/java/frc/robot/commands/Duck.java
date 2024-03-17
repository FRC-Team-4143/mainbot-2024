package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;

public class Duck extends Command {

    public Duck() {
        addRequirements(ShooterSubsystem.getInstance());
    }
    @Override
    public void initialize() {
        ShooterSubsystem.getInstance().setShootMode(ShootMode.DUCK);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
