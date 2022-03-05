package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class ShuffleboardControl {
    public static ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Station");
    public static ShuffleboardTab sensorsTab = Shuffleboard.getTab("Sensors");

    public ShuffleboardControl(){
    }

    public static void addDriverInfo(){
    }

    public static void addSensor(String title, boolean value){
        sensorsTab.add(title, value);
    }
}
