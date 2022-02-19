package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class PipelineCamera extends CommandBase{

    /**
     * Enables the pipeline camera for the limelight on command run
     */
    public PipelineCamera() {
    }

    @Override
    public void initialize() {
        Limelight.setCameraMode(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
