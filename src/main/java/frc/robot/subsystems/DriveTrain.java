// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drive;

public class DriveTrain extends SubsystemBase {
    AHRS navx;

    DifferentialDriveOdometry odometry;
    
    MotorControllerGroup leftMotorGroup;
    MotorControllerGroup rightMotorGroup;

    DifferentialDrive drive;

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

        navx = new AHRS(Port.kMXP);
        odometry = new DifferentialDriveOdometry(navx.getRotation2d());
    }

    public void drive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    public void driveByVoltage(double leftVoltage, double rightVoltage){
        leftMotorGroup.setVoltage(leftVoltage);
        rightMotorGroup.setVoltage(rightVoltage);
        drive.feed();
    }

    public void resetAngle(){
        navx.reset();
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        double leftMetersPerSecond = 0;
        double rightMetersPerSecond = 0;
        return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double leftDistanceMeters = 0;
        double rightDistanceMeters = 0;
        odometry.update(navx.getRotation2d(), leftDistanceMeters, rightDistanceMeters);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
