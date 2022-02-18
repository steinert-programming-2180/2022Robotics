package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.Constants.*;

// I would like to preface this by saying that it is almost 11 PM when writing this documentation

public class Limelight extends SubsystemBase{

    // Gets our Limelight NetworkTable, api located here: https://docs.limelightvision.io/en/latest/networktables_api.html
    // Uses a static reference so that it can be used in static methods
    private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    // Gets the NetworkTableEntries for x angle from center, y angle from center,
    // Area of contour, and number of valid targets, respectively
    private NetworkTableEntry tx = limelight.getEntry("tx");
    private NetworkTableEntry ty = limelight.getEntry("ty");
    private NetworkTableEntry ta = limelight.getEntry("ta");
    private NetworkTableEntry tv = limelight.getEntry("tv");

    // Gets the NetworkTableEntries for cameraMode, ledMode, and pipelineNumber, respectively
    // Uses a static reference so that they can be used in static methods
    private static NetworkTableEntry camMode = limelight.getEntry("camMode");
    private static NetworkTableEntry ledMode = limelight.getEntry("ledMode");
    private static NetworkTableEntry pipelineEntry = limelight.getEntry("pipeline");

    /**
     * Limelight class for command usage, non-static usage pretty much entirely depricated.
    */
    public Limelight () {}

    /**
     * Calculates the horizontal distance from target, as described by {@link https://docs.limelightvision.io/en/latest/cs_estimating_distance.html}
     * 
     * @return Distance from target, in meters because we are scientists first and Americans second.
     */
    public double getHorizontalDistance() {

        // Gets if we have a valid target
        double v = tv.getDouble(0);

        // If no valid target, return -1
        if(v < 1) {
            return -1;
        }
        
        // Gets our angle from horizontal by adding angle of limelight to angle offset
        double theta = Math.toRadians(ty.getDouble(0) + LimelightConstants.LIME_ANGLE);

        // theta = 0 implies tan(theta) = 0, so to avoid division
        // by zero, return -1 if theta = 0
        if(theta == 0) {
            return -1;
        }

        // Gets delta height from top of target to where the limelight is mounted
        double height = Constants.inchesToMeters(FieldConstants.TARGET_MAX_HEIGHT - LimelightConstants.LIME_HEIGHT);

        // Returns distance by using trigonometry
        return height / Math.tan(theta);
    }

    /**
     * Calculates the straight-on distance from target to limelight, inspired by {@link https://docs.limelightvision.io/en/latest/cs_estimating_distance.html}
     * 
     * @return Distance from target, in meters because we are scientists first and Americans second.
     */
    public double getStraightDifference() {
        // Gets if we have a valid target
        double v = tv.getDouble(0);

        // If no valid target, return -1
        if(v < 1) {
            return -1;
        }
        
        // Gets our angle from horizontal by adding angle of limelight to angle offset
        double theta = Math.toRadians(ty.getDouble(0) + LimelightConstants.LIME_ANGLE);

        // theta = 0 implies sin(theta) = 0, so to avoid division
        // by zero, return -1 if theta = 0
        if(theta == 0) {
            return -1;
        }

        // Gets delta height from top of target to where the limelight is mounted
        double height = Constants.inchesToMeters(FieldConstants.TARGET_MAX_HEIGHT - LimelightConstants.LIME_HEIGHT);

        // Returns distance by using trigonometry
        return height / Math.sin(theta);
    }
    /**
     * Puts x angle of target, y angle of target, and area of target. If distance is valid, put that, too.
     */

    public void putItems() {

        // Gets x, y, area. If invalid, defaults to 0.0
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        
        // Puts x, y, area to SmartDashboard
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        // Gets the horizontal distance from the target
        double dist = getHorizontalDistance();

        // If this distance is positive, i.e. didn't trigger a failsafe, put it to SmartDashboard
        if(dist >=0) {
            SmartDashboard.putNumber("Distance (inches)", dist);
        }

    }


    /**
     * Sets the current pipeline to the parameter.
     * 
     * @param pipeline The pipeline we wish to use
     */
    public static void setPipeline(double pipeline) {
        pipelineEntry.setNumber(pipeline);
    }

    /**
     * Returns the current pipeline in use.
     * 
     * @return The pipeline in use right now
     */

    public static double getPipeline() {
        return (double) pipelineEntry.getNumber(0);
    }

    /**
     * Sets the current camera mode to the camera, 
     * 0 = pipeline mode,
     * 1 = driver mode
     * 
     * @param mode The camera mode we wish to use
     */
    public static void setCameraMode(double mode) {
        camMode.setNumber(mode);
    }

    /**
     * Gets the current camera mode.
     * 
     * @return The current camera mode
     */

    public static double getCameraMode() {
        return (double) camMode.getNumber(0);
    }

    /**
     * Sets the LED mode of the limelight,
     * 0 = pipeline camera,
     * 1 = force off, 
     * 2 = force blink,
     * 3 = force on
     * 
     * @param mode The mode of the LED we want to use
     */
    public static void setLightsMode(double mode) {
        ledMode.setNumber(mode);
    }

    /**
     * Gets the current LED mode of the limelight.
     * 
     * @return The current LED mode,
     * 0 = pipeline camera,
     * 1 = force off, 
     * 2 = force blink,
     * 3 = force on
     */
    public static double getLightMode() {
        return (double) ledMode.getNumber(0);
    }

    /**
     * Returns the limelight table to be accessed elsewhere, if necessary.
     * 
     * @return The NetworkTable for the limelight, API found here {@link https://docs.limelightvision.io/en/latest/networktables_api.html}
     */
    public static NetworkTable getLimelightTable() {
        return limelight;
    }

    /**
     * Swaps the LED Status between 0 = pipeline and 1 = force off.
     * 
     */
    public static void swapLights() {
        // The ? statement handles the if our current LED status is not 0 or 1
        setLightsMode(1 - (getLightMode() < 2 ? getLightMode() : 1));
    }

    /**
     * Swaps the Camera Status between 0 = pipeline camera and 1 = drive camera
     */
    public static void swapCamera() {
        setCameraMode(1 - getCameraMode());
    }
}