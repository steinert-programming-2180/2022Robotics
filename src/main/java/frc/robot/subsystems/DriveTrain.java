// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drive;

public class DriveTrain extends SubsystemBase {
   
    DifferentialDrive drive;
    /** Creates a new ExampleSubsystem. */
    public DriveTrain() {
        int amountOfLeftMotors = Constants.Drive.leftMotorPorts.length;
        int amountOfRightMotors = Constants.Drive.rightMotorPorts.length;
        WPI_TalonSRX[] leftMotors = new WPI_TalonSRX[amountOfLeftMotors];
        WPI_TalonSRX[] rightMotors = new WPI_TalonSRX[amountOfRightMotors];

        // Make Left Talons from the ports
        for (int i = 0; i < amountOfLeftMotors; i++)
            leftMotors[i] = new WPI_TalonSRX(Constants.Drive.leftMotorPorts[i]);

        // Make Right Talons from the ports
        for (int i = 0; i < amountOfRightMotors; i++)
            rightMotors[i] = new WPI_TalonSRX(Constants.Drive.rightMotorPorts[i]);

        // Put motors into their own groups
        MotorControllerGroup leftMotorGroup = new MotorControllerGroup(leftMotors);
        MotorControllerGroup rightMotorGroup = new MotorControllerGroup(rightMotors);
        // Invert to drive propery
        rightMotorGroup.setInverted(true);

        // Create the Differential Drive to drive the robot
        drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

    }

    public void drive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
