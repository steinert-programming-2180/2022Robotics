package frc.robot.commands.limelight;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class PipelineCamera extends CommandBase{
    private final Limelight m_limelight;

    public PipelineCamera(Limelight lime) {
        m_limelight = lime;

        addRequirements(m_limelight);
    }

    @Override
    public void initialize() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    }



}
