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

        public static final double kP = 42.42; // was 0.04
        public static final double kI = 0;
        public static final double kD = 2.8747; // was 0.005

        public static final double kS = 0.16283;
        public static final double kV = 0.67659;
        public static final double kA = 0.122;

        public static final double leftKp = 0.49699;
        public static final double leftKi = 0;
        public static final double leftKd = 0;

        public static final double rightKp = 0.49699;
        public static final double rightKi = 0;
        public static final double rightKd = 0;

        public static final double b = 2;
        public static final double zeta = 0.7;

        public static final double trackWidth = 1;

        public static final int highGearSolenoid = 2;
        public static final int lowGearSolenoid = 0;

        public static final double lowGearRatio = 12.03;
        public static final double highGearRatio = 5.56;
        public static final double wheelDiameter = InchesToMeters(4);
           
    }
}
