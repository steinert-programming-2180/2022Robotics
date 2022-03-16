package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutonomousConstants;

public final class ShuffleboardControl {
    public static ShuffleboardTab smartDashboardTab = Shuffleboard.getTab("SmartDashboard");
    private static SendableChooser<Integer> autoChooser = new SendableChooser<Integer>();


    /*
    - sensors of beam breaks
    - limit switch
    - encoder value (right is the only one)
    - speed of the shooter (you can change)
    */

    public static void addDriverInfo(){
    }

    public static void initializeAutonomousChooser(){
        String[] options = AutonomousConstants.autonomousOptions;
        for(int i  = 0; i < options.length; i++){
            if(i == AutonomousConstants.defaultAutonomous) {
                autoChooser.setDefaultOption(options[i], i);
            } else {
                autoChooser.addOption(options[i], i);
            }
        }
        // TODO: insert paths
        SmartDashboard.putData("Auto", autoChooser);
    }

    public static int getAutonomousMode(){
        return autoChooser.getSelected();
    }

    public static void addSensor(String title, boolean value){
        smartDashboardTab.add(title, value);
    }
}
