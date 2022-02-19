package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class SwapLights extends CommandBase{

    Limelight m_limelight;

    /**
     * Swaps between light modes on command run
     */
    public SwapLights(Limelight lime) {
        m_limelight = lime;

        addRequirements(lime);
    }

    @Override
    public void initialize() {
        m_limelight.swapLights();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
