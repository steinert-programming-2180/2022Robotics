// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;

public class DriveTrain extends SubsystemBase {
    AHRS navx;

    DifferentialDriveOdometry odometry;
    
    MotorControllerGroup leftMotorGroup;
    MotorControllerGroup rightMotorGroup;

    DifferentialDrive drive;
    Compressor compressor;

    CANSparkMax[] leftMotors;
    CANSparkMax[] rightMotors;

    /** Creates a new ExampleSubsystem. */
    public DriveTrain() {
        int amountOfLeftMotors = Drive.leftMotorPorts.length;
        int amountOfRightMotors = Drive.rightMotorPorts.length;
        
        leftMotors = new CANSparkMax[amountOfLeftMotors];
        rightMotors = new CANSparkMax[amountOfRightMotors];

        // Make Left Talons from the ports
        for (int i = 0; i < amountOfLeftMotors; i++)
            leftMotors[i] = new CANSparkMax(Drive.leftMotorPorts[i], MotorType.kBrushless);

        // Make Right Talons from the ports
        for (int i = 0; i < amountOfRightMotors; i++)
            rightMotors[i] = new CANSparkMax(Drive.rightMotorPorts[i], MotorType.kBrushless);

        // Put motors into their own groups
        leftMotorGroup = new MotorControllerGroup(leftMotors);
        rightMotorGroup = new MotorControllerGroup(rightMotors);

        // Invert to drive propery
        leftMotorGroup.setInverted(true);

        // Create the Differential Drive to drive the robot
        drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
        //compressor = new Compressor(PneumaticsModuleType.REVPH);
        navx = new AHRS(Port.kMXP);
        odometry = new DifferentialDriveOdometry(navx.getRotation2d());
    }

    public void drive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);

        // TODO: only for testing; get rid of this later
        // for(CANSparkMax i : leftMotors) i.set(leftSpeed);
        // for(CANSparkMax i : rightMotors) i.set(rightSpeed);
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
