package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class Limelight extends SubsystemBase{

    private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    private NetworkTableEntry tx = limelight.getEntry("tx");
    private NetworkTableEntry ty = limelight.getEntry("ty");
    private NetworkTableEntry ta = limelight.getEntry("ta");
    private NetworkTableEntry tv = limelight.getEntry("tv");

    
    private static NetworkTableEntry camMode = limelight.getEntry("camMode");
    private static NetworkTableEntry ledMode = limelight.getEntry("ledMode");
    private static NetworkTableEntry pipelineEntry = limelight.getEntry("pipeline");


    public Limelight () {}

    public double getDistance() {
        double v = tv.getDouble(0);
        if(v < 1) {
            return -1;
        }
        double theta = ty.getDouble(0) + LimelightConstants.LIME_ANGLE;
        double height = Constants.inchesToMeters(FieldConstants.TARGET_MAX_HEIGHT - LimelightConstants.LIME_HEIGHT);

        if(theta == 0) {
            return -1;
        }
        return height / Math.tan(Math.toRadians(theta));
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

    public static void setPipeline(double pipeline) {
        pipelineEntry.setNumber(pipeline);
    }

    public static double getPipeline() {
        return (double) pipelineEntry.getNumber(0);
    }

    public static void setCameraMode(double mode) {
        camMode.setNumber(mode);
    }

    public static double getCameraMode() {
        return (double) camMode.getNumber(0);
    }

    public static void setLedMode(double mode) {
        ledMode.setNumber(mode);
    }

    public static double getLedMode() {
        return (double) ledMode.getNumber(0);
    }

    public static NetworkTable getLimelightTable() {
        return limelight;
    }

    public static void swapLed() {
        setLedMode(1 - getLedMode());
    }

    public static void swapCamera() {
        setCameraMode(1 - getCameraMode());
    }
}