package frc.robot.commands.limelight;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class EnableLights extends CommandBase{

    public EnableLights() {}

    @Override
    public void initialize() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    }

    

}
