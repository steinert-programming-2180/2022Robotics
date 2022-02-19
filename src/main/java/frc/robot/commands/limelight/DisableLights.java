package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class DisableLights extends CommandBase{

    Limelight m_limelight;

    /**
     * Disables the lights of the limelight on Command run
     */
    public DisableLights(Limelight lime) {
        m_limelight = lime;

        addRequirements(lime);
    }

    @Override
    public void initialize() {
        m_limelight.setLightsMode(1);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
