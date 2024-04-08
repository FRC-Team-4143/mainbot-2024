package frc.lib;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class SwitchCommand extends Command{
    private Command c1_;
    private Command c2_;
    private BooleanSupplier switch_condition_;
    private boolean c1_active = false;

    public SwitchCommand(Command c1, Command c2, BooleanSupplier switchCondition) {
        c1_ = c1;
        c2_ = c2;
        switch_condition_ = switchCondition;
    }

    public void initialize() {
        c1_.initialize();
        c1_active = false;
    }

    public void execute() {
        if(c1_active) {
            c1_.execute();
            if(switch_condition_.getAsBoolean()){
                c1_active = false;
                c1_.end(true);
                c2_.initialize();
            }
        } else {
            c2_.execute();
        }
    }

    public void end(boolean interrupted) {
        if(c1_active) {
            c1_.end(interrupted);
        } else {
            c2_.end(interrupted);
        }
    }

    public boolean isFinished() {
        if(c1_active) {
            return c1_.isFinished();
        } else {
            return c2_.isFinished();
        } 
    }
}