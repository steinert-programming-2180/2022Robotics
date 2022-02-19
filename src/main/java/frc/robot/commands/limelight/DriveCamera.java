package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class DriveCamera extends CommandBase{

    Limelight m_limelight;

    /**
     * Enables the driver camera for the limelight on command run
     */
    public DriveCamera(Limelight lime) {
        m_limelight = lime;

        addRequirements(lime);
    }

    @Override
    public void initialize() {
        m_limelight.setCameraMode(1);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
