package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;

public class WristClimb extends Command {

    @Override
    public void initialize() {
        ShooterSubsystem.getInstance().setShootMode(ShootMode.CLIMB);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
