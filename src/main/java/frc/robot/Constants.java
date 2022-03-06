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
        public static final double shooterSpeed = 0.2;

        public static final int bottomFlywheelPort = 12;
        public static final int topFlywheelPort = 11;
    }

    public final class ArmConstants{
        public static final int leftArmRaiserPort = 13;
        public static final int rightArmRaiserPort = 14;

        public static final int lowerLimitSwitchPort = 0;
        public static final int topLimitSwitchPort = 2;

        public static final double armSpeed = 0.3;
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
        public static final String[] autonomousOptions = {"None", "Drive Backward", "Simple Auto"};
        public static final int defaultAutonomous = 1; // This is the index of the default auto
        
        // Times are in SECONDS
        public static final double shooterTime = 3;
        public static final double conveyorTime = 3;
        public static final double driveTime = 5;
    }

    public static final int PneumaticHubPort = 20;
    public static final int PowerDistributionPort = 22;

    public static double inchesToMeters(double inches) { return inches * 0.0254; }
    
    // EVERYTHING IS IN METERS!!!
    public static final class DriveConstants{
        public static final double autonomousSpeed = 0.4;

        public static final double initialSpeedLimit = 0.7;
        public static final double secondSpeedLimit = 0.8;

        public static final int[] leftMotorPorts = {1, 2, 3};
        public static final int[] rightMotorPorts = {4, 5, 6};

        public static final int highGearSolenoid = 2;
        public static final int lowGearSolenoid = 0;

        public static final double kP = 23.741;
        public static final double kI = 0;
        public static final double kD = 0.6788;

        public static final double kS = 0.13112;
        public static final double kV = 0.12103;
        public static final double kA = 0.012125;

        // These are for velocity
        public static final double leftKp = 0;
        public static final double leftKi = 0;
        public static final double leftKd = 0;

        public static final double rightKp = 0;
        public static final double rightKi = 0;
        public static final double rightKd = 0;

        public static final double b = 0;
        public static final double zeta = 0;

        public static final double wheelDiameter = inchesToMeters(4);
        public static final double trackWidth = 0.60516;
        public static final double highGearRatio = 5.56/1.0;
        public static final double lowGearRatio = 12.03/1.0;
    }
}
