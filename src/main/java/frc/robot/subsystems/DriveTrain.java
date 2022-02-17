// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drive;

public class DriveTrain extends SubsystemBase {
   
    DifferentialDrive drive;
    MotorControllerGroup leftMotorGroup;
    MotorControllerGroup rightMotorGroup;

    double kS = 1;
    double kV = 1;
    double kA = 1;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    /** Creates a new ExampleSubsystem. */
    public DriveTrain() {
        int amountOfLeftMotors = Drive.leftMotorPorts.length;
        int amountOfRightMotors = Drive.rightMotorPorts.length;
        WPI_TalonSRX[] leftMotors = new WPI_TalonSRX[amountOfLeftMotors];
        WPI_TalonSRX[] rightMotors = new WPI_TalonSRX[amountOfRightMotors];

        // Make Left Talons from the ports
        for (int i = 0; i < amountOfLeftMotors; i++)
            leftMotors[i] = new WPI_TalonSRX(Drive.leftMotorPorts[i]);

        // Make Right Talons from the ports
        for (int i = 0; i < amountOfRightMotors; i++)
            rightMotors[i] = new WPI_TalonSRX(Drive.rightMotorPorts[i]);

        // Put motors into their own groups
        leftMotorGroup = new MotorControllerGroup(leftMotors);
        rightMotorGroup = new MotorControllerGroup(rightMotors);

        // Invert to drive propery
        leftMotorGroup.setInverted(true);

        // Create the Differential Drive to drive the robot
        drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

    }

    public void drive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    public void driveWithFeedForward(double leftVelocity, double rightVelocity){
        leftMotorGroup.setVoltage(feedforward.calculate(leftVelocity));
        rightMotorGroup.setVoltage(feedforward.calculate(rightVelocity));
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
