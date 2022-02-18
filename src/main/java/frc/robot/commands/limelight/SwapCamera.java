package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class SwapCamera extends CommandBase{

    /**
     * Swaps between camera modes on command run
     */
    public SwapCamera() {}

    @Override
    public void initialize() {
        Limelight.swapCamera();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
