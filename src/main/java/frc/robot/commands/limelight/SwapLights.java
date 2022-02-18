package frc.robot.commands.limelight;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwapLights extends CommandBase{

    NetworkTableEntry ledMode;

    public SwapLights() {
        ledMode = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode");
    }

    @Override
    public void initialize() {
        double currentLedMode = (double) ledMode.getNumber(0);
        ledMode.setNumber(1 - currentLedMode);
    }

    
    @Override
    public boolean isFinished() {
        return true;
    }
}
