package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseEstimator;

public class SetIgnoreVision extends Command {
    private boolean want_ignore_ = false;

    public SetIgnoreVision(boolean ignore) {
        want_ignore_ = ignore;
    }

    @Override
  public void initialize() {
    PoseEstimator.getInstance().setIgnoreVision(want_ignore_);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
