package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class EnableLights extends CommandBase{

    Limelight m_limelight;

    /**
     * Enables the lights of the limelight on command run
     */
    public EnableLights(Limelight lime) {
        m_limelight = lime;

        addRequirements(lime);
    }

    @Override
    public void initialize() {
        m_limelight.setLightsMode(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
