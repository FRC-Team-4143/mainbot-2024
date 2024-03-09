package frc.robot.commands.climberSequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.MailmanSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbTarget;
import frc.robot.subsystems.MailmanSubsystem.HeightTarget;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;

public class DeclimbState extends Command {
  /** Creates a new ClimbState. */
  public DeclimbState() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    MailmanSubsystem.getInstance().setHeight(HeightTarget.HOME);
    ShooterSubsystem.getInstance().setShootMode(ShootMode.CLIMB);
    ClimberSubsystem.getInstance().setHeight(ClimbTarget.HOME, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ClimberSubsystem.getInstance().raiseHeightTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (ClimberSubsystem.getInstance().getTargetHeight() == ClimberConstants.MAX_HEIGHT);
  }
}
