package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class DisableLights extends CommandBase{

    /**
     * Disables the lights of the limelight on Command run
     */
    public DisableLights() {}

    @Override
    public void initialize() {
        Limelight.setLightsMode(1);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
