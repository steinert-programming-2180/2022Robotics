package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RaiseArm extends CommandBase{
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    Arm arm;

    public RaiseArm(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.initialize();
    }

    @Override
    public void execute() {
        arm.raiseArm();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopArm();
    }

    @Override
    public boolean isFinished() {
        return arm.hasReachedUpperLimit();
    }
}
