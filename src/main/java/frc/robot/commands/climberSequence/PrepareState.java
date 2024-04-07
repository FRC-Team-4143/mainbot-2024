package frc.robot.commands.climberSequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.MailmanSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbTarget;
import frc.robot.subsystems.MailmanSubsystem.HeightTarget;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;

public class PrepareState extends Command {
  /** Creates a new PrepareState. */
  private int timer; 
  public PrepareState() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    MailmanSubsystem.getInstance().setRollerBump();
    MailmanSubsystem.getInstance().setHeight(HeightTarget.TRAP);
    ClimberSubsystem.getInstance().setHeight(ClimbTarget.MAX, 0);
    ShooterSubsystem.getInstance().setShootMode(ShootMode.CLIMB);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    MailmanSubsystem.getInstance().setRollerStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 5;
  }
}
