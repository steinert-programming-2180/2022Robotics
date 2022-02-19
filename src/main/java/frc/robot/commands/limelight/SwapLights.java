package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class SwapLights extends CommandBase{

    /**
     * Swaps between light modes on command run
     */
    public SwapLights() {}

    @Override
    public void initialize() {
        Limelight.swapLights();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
