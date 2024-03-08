package frc.robot.commands.climberSequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbTarget;
import frc.robot.subsystems.MailmanSubsystem;
import frc.robot.subsystems.MailmanSubsystem.HeightTarget;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;

public class EngageState extends Command {
  /** Creates a new EngageState. */
  public EngageState() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ClimberSubsystem.getInstance().setHeight(ClimbTarget.MAX);
    ShooterSubsystem.getInstance().setShootMode(ShootMode.CLIMB);
    MailmanSubsystem.getInstance().setHeight(HeightTarget.HOME);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}