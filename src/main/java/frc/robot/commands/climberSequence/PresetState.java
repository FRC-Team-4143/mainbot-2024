package frc.robot.commands.climberSequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.MailmanSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbTarget;
import frc.robot.subsystems.MailmanSubsystem.HeightTarget;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;

public class PresetState extends Command {
  /** Creates a new CleanAllPickups. */
  public PresetState() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ClimberSubsystem.getInstance().setHeight(ClimbTarget.HOME, 0);
    ShooterSubsystem.getInstance().setShootMode(ShootMode.IDLE);
    MailmanSubsystem.getInstance().setHeight(HeightTarget.HOME);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ClimberSubsystem.getInstance().setClimbSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ClimberSubsystem.getInstance().atHeight() && (ClimberSubsystem.getInstance().getTargetHeight() == ClimberConstants.HOME_HEIGHT);
  }
}
