package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.*;

public class Limelight extends SubsystemBase{

    private NetworkTable limelight;
    private NetworkTableEntry tx, ty, ta, tv;

    public Limelight () {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        limelight.getEntry("pipeline").setNumber(0);
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");
        tv = limelight.getEntry("tv");
    }

    public double getDistance() {
        double v = tv.getDouble(0);
        if(v < 1) {
            return -1;
        }
        double theta = ty.getDouble(0) + LimelightConstants.LIME_ANGLE;
        double height = FieldConstants.TARGET_MAX_HEIGHT - LimelightConstants.LIME_HEIGHT;

        if(theta == 0) {
            return -1;
        }
        return height * Math.cot(Math.toRadians(theta));
    }

    public void putItems() {
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        double dist = getDistance();
        if(dist >=0) {
            SmartDashboard.putNumber("Distance (inches)", dist);
        }

    }

    public void setPipeline(int pipeline) {
        NetworkTableEntry pipelineEntry = limelight.getEntry("pipeline");
        pipelineEntry.setNumber(pipeline);
    }

    public double getPipeline() {
        return (double) limelight.getEntry("pipeline").getNumber(0);
    }
}