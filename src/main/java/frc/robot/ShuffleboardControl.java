package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutonomousConstants;

public final class ShuffleboardControl {
    public static ShuffleboardTab developmentTab = Shuffleboard.getTab("Development");
    private static SendableChooser<Integer> autoChooser = new SendableChooser<Integer>();

    private static ArrayList<NetworkTableEntry> developmentTabEntries = new ArrayList<NetworkTableEntry>();

    public static void initializeCameraServer(){
        CameraServer.startAutomaticCapture();
    }

    public static void initializeAutonomousChooser(){
        String[] options = AutonomousConstants.autonomousOptions;

        // Adds all auto paths to ShuffleBoard; Our default is defaulted
        for(int i  = 0; i < options.length; i++){
            if(i == AutonomousConstants.defaultAutonomous) 
                autoChooser.setDefaultOption(options[i], i);
            else
                autoChooser.addOption(options[i], i);
        }

        SmartDashboard.putData("Auto", autoChooser);
    }

    public static int getAutonomousMode(){
        return autoChooser.getSelected();
    }

    public static void addToDevelopment(String title, Object value){
        NetworkTableEntry existingEntry = getEntry(title);

        if(existingEntry != null){
            existingEntry.setValue(value);
        } else {
            NetworkTableEntry newEntry = developmentTab.add(title, value).getEntry();
            developmentTabEntries.add(newEntry);
        }
    }

    static NetworkTableEntry getEntry(String entryTitle){
        String path = "/Shuffleboard/Development/";

        for(NetworkTableEntry i : developmentTabEntries){
            String title = i.getName().substring(path.length());
            if(title.equals(entryTitle)) return i;
        }
        return null;
    }
}
