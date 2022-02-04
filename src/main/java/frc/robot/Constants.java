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
    static final int leftJoystickPort = 0;
    static final int rightJoystickPort = 1;
    
    
    public static final class Drive{
        static final int[] leftMotorPorts = {7, 12, 15};
        static final int[] rightMotorPorts = {1, 2, 3};

        static final double lowModifier = 0.75;
        static final double medModifier = 0.85;
        static final double highModifier = 1;
    }
}
