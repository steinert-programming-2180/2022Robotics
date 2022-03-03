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
        public static final int bottomFlywheelPort = 12;
        public static final int topFlywheelPort = 11;
        public static final double shooterSpeed = 1.0/3;
    }

    public final class ConveyorConstants{
        public static final int entranceConveyorPort = 10;
        public static final int exitConveyorPort = 9;
        public static final double conveyorSpeed = 1;
    }

    public final class IntakeConstants{
        public static final int leftIntakePort = 8;
        public static final int rightIntakePort = 7;
    }

    public final class IO{
        public static final int xboxPort = 0;
    }
    
    public static final int leftJoystickPort = 1;
    public static final int rightJoystickPort = 2;

    public static final int PneumaticHubPort = 20;
    public static final int PowerDistributionPort = 22;

    public static double InchesToMeters(double inches) { return inches * 0.0254; }
    
    // EVERYTHING IS IN METERS!!!
    public static final class Drive{
        public static final int[] leftMotorPorts = {1, 2, 3};
        public static final int[] rightMotorPorts = {4, 5, 6};

        static final double lowModifier = 0.75;
        static final double medModifier = 0.85;
        static final double highModifier = 1;

        public static final double kP = 0.04;
        public static final double kI = 0;
        public static final double kD = 0.005;

        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double leftKp = 0;
        public static final double leftKi = 0;
        public static final double leftKd = 0;

        public static final double rightKp = 0;
        public static final double rightKi = 0;
        public static final double rightKd = 0;

        public static final double b = 0;
        public static final double zeta = 0;

        public static final double trackWidth = 1;

        public static final int highGearSolenoid = 2;
        public static final int lowGearSolenoid = 0;
    }
}
