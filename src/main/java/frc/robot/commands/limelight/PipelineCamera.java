package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class PipelineCamera extends CommandBase{

    Limelight m_limelight;

    /**
     * Enables the pipeline camera for the limelight on command run
     */
    public PipelineCamera(Limelight lime) {
        m_limelight = lime;

        addRequirements(lime);
    }

    @Override
    public void initialize() {
        m_limelight.setCameraMode(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
