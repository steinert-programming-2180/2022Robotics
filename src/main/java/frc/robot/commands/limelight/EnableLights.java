package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class EnableLights extends CommandBase{

    /**
     * Enables the lights of the limelight on command run
     */
    public EnableLights() {}

    @Override
    public void initialize() {
        Limelight.setLightsMode(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
