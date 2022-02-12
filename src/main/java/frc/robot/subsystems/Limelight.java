package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.*;

public class Limelight extends SubsystemBase{

    private NetworkTable limelight;
    private NetworkTableEntry tx, ty, ta;

    public Limelight () {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        limelight.getEntry("pipeline").setNumber(0);
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");
    }

    public double getDistance() {
        double tv = limelight.getEntry("tv").getDouble(0);
        if(tv == 0) {
            return -1;
        }
        double theta = tx.getDouble(0) + LimelightConstants.LIME_ANGLE;
        double height = FieldConstants.TARGET_MAX_HEIGHT - LimelightConstants.LIME_ANGLE;

        return height / Math.tan(Math.toRadians(theta));
    }

    public void putItems() {
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }

    public void setPipeline(int pipeline) {
        NetworkTableEntry pipelineEntry = limelight.getEntry("pipeline");
        pipelineEntry.setNumber(pipeline);
    }

    public double getPipeline() {
        return (double) limelight.getEntry("pipeline").getNumber(0);
    }
}
