package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class SwapCamera extends CommandBase{

    Limelight m_limelight;

    /**
     * Swaps between camera modes on command run
     */
    public SwapCamera(Limelight lime) {
        m_limelight = lime;

        addRequirements(lime);
    }

    @Override
    public void initialize() {
        m_limelight.swapCamera();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
