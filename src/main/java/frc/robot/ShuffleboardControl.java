package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class ShuffleboardControl {
    public static ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Station");
    public static ShuffleboardTab sensorsTab = Shuffleboard.getTab("Sensors");

    /*
    - sensors of beam breaks
    - limit switch
    - encoder value (right is the only one)
    - speed of the shooter (you can change)
    */

    public ShuffleboardControl(){
    }

    public static void addDriverInfo(){
    }

    public static void addSensor(String title, boolean value){
        sensorsTab.add(title, value);
    }
}
