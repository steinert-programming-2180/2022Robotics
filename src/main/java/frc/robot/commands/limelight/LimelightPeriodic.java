package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class LimelightPeriodic extends CommandBase{
    private final Limelight m_limelight;

    /**
     * The periodic command for the Limelight, Calls putItems
     */
    public LimelightPeriodic(Limelight lime) {
        m_limelight = lime;
        addRequirements(m_limelight);
    }

    @Override
    public void execute() {
        m_limelight.putItems();
    }


}
