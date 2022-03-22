// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class ShooterConstants{
        public static final double shooterRPM = 1750;

        public static final int bottomFlywheelPort = 12;
        public static final int topFlywheelPort = 11;

        public static final double kS = 0.17096;
        public static final double kV = 0.12577;
        public static final double kA = 0.0086187;
    }

    public final class ArmConstants{
        public static final int leftArmRaiserPort = 13;
        public static final int rightArmRaiserPort = 14;

        public static final int lowerLimitSwitchPort = 0;
        public static final int potentiometerPort = 0;

        public static final double upSpeed = 0.22;
        public static final double downSpeed = -0.4;

        public static final int maxEncoderVal = 73;
    }

    public static final class ConveyorConstants{
        public static final double autonomousTime = 3; // in SECONDS
        public static final double conveyorSpeed = 1;

        public static final int entranceConveyorPort = 10;
        public static final int exitConveyorPort = 9;

        public static final int entranceBeamBreakPort = 2;
        public static final int exitBeamBreakPort = 3;

        public enum ConveyorSection {
            ENTRANCE,
            EXIT
        }
    }

    public final class IntakeConstants{
        public static final int leftIntakePort = 8;
        public static final int rightIntakePort = 7;

        public static final int extendSolenoid = 1;
        public static final int retractSolenoid = 3;
    }

    public final class IO{
        public static final int xboxPort = 0;
        public static final int leftJoystickPort = 1;
        public static final int rightJoystickPort = 2;
    }

    public static final class AutonomousConstants {
        public static final String[] autonomousOptions = {"None", "Zone 1 - 2", "Zone 2 - 2", "Zone 2 - 3", "Drive Out"};
        public static final int defaultAutonomous = 2; // This is the index of the default auto
        
        // Times are in SECONDS
        public static final double shooterTime = 2;
        public static final double conveyorTime = 2;
        public static final double driveTime = 7;
    }

    public static final int PneumaticHubPort = 20;
    public static final int PowerDistributionPort = 22;

    public static double inchesToMeters(double inches) { return inches * 0.0254; }
    
    // EVERYTHING IS IN METERS PER SECONDS!!!
    public static final class DriveConstants{
        public static final double autonomousSpeed = 0.7;

        public static final double initialSpeedLimit = 0.7;
        public static final double secondSpeedLimit = 0.8;

        public static final int[] leftMotorPorts = {1, 2, 3};
        public static final int[] rightMotorPorts = {4, 5, 6};

        public static final int highGearSolenoid = 2;
        public static final int lowGearSolenoid = 0;

        // PID for velocity
        public static final double kP = 1.3935;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0.17341;
        public static final double kV = 2.1344;
        public static final double kA = 0.31494;

        public static final double trackWidth = 0.6477;
        public static final double lowGearRatio = 12.03;
        public static final double highGearRatio = 5.56;
        public static final double wheelDiameter = inchesToMeters(4);
           
    }
}
