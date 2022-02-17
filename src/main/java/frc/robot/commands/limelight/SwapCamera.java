package frc.robot.commands.limelight;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class SwapCamera extends CommandBase{
    private final Limelight m_limelight;
    private final NetworkTableEntry camMode;

    public SwapCamera(Limelight lime) {
        m_limelight = lime;

        addRequirements(m_limelight);
        camMode = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode");
    }

    @Override
    public void initialize() {
        double currentCamMode = (double) camMode.getNumber(0);
        camMode.setNumber(1 - currentCamMode);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
