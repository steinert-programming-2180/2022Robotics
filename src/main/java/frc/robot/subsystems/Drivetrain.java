// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
public class Drivetrain extends SubsystemBase {
    AHRS navx;

    DifferentialDriveOdometry odometry;

    MotorControllerGroup leftMotorGroup;
    MotorControllerGroup rightMotorGroup;

    DifferentialDrive drive;
    Compressor compressor;

    CANSparkMax[] leftMotors;
    CANSparkMax[] rightMotors;

    boolean inHighGear = true;
    
    RelativeEncoder leftEncoder;
    RelativeEncoder righEncoder;

    DoubleSolenoid shifters;

    /** Creates a new ExampleSubsystem. */
    public Drivetrain() {
        setupMotorArrays();
        leftMotorGroup = new MotorControllerGroup(leftMotors);
        rightMotorGroup = new MotorControllerGroup(rightMotors);

        // Invert to drive propery
        leftMotorGroup.setInverted(false);
        rightMotorGroup.setInverted(true);

        // Create the Differential Drive to drive the robot
        drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

        navx = new AHRS(Port.kMXP);
        odometry = new DifferentialDriveOdometry(navx.getRotation2d());
        leftEncoder = leftMotors[0].getEncoder();
        righEncoder = rightMotors[0].getEncoder();

        shifters = new DoubleSolenoid(Constants.PneumaticHubPort, PneumaticsModuleType.REVPH, DriveConstants.highGearSolenoid, DriveConstants.lowGearSolenoid);
        inHighGear = getGear();
    }

    // True if high; False if low;
    public boolean getGear(){
        return shifters.get() == Value.kForward;
    }

    public double getCurrentGearRatio(){
        return getGear() ? DriveConstants.highGearRatio : DriveConstants.lowGearRatio;
    }

    public double rpmToVelocity(double rpm){
        return rpm / getCurrentGearRatio() * (1.0/60.0);
    }

    public double getWheelCircumference(){
        return DriveConstants.wheelDiameter * Math.PI;
    }

    private void setupMotorArrays() {
        int amountOfLeftMotors = DriveConstants.leftMotorPorts.length;
        int amountOfRightMotors = DriveConstants.rightMotorPorts.length;

        leftMotors = new CANSparkMax[amountOfLeftMotors];
        rightMotors = new CANSparkMax[amountOfRightMotors];

        // Make Left Sparks from the ports
        for (int i = 0; i < amountOfLeftMotors; i++)
            leftMotors[i] = new CANSparkMax(DriveConstants.leftMotorPorts[i], MotorType.kBrushless);

        // Make Right Sparks from the ports
        for (int i = 0; i < amountOfRightMotors; i++)
            rightMotors[i] = new CANSparkMax(DriveConstants.rightMotorPorts[i], MotorType.kBrushless);
    }

    public void drive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    public void driveByVoltage(double leftVoltage, double rightVoltage) {
        leftMotorGroup.setVoltage(leftVoltage);
        rightMotorGroup.setVoltage(rightVoltage);
        drive.feed();
    }

    public void highGear() {
        shifters.set(Value.kForward);
    }

    public void lowGear() {
        shifters.set(Value.kReverse);
    }

    public void resetAngle() {
        navx.reset();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    
    // TODO: get actual meters per second
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        double leftMetersPerSecond = rpmToVelocity( leftEncoder.getVelocity() );
        double rightMetersPerSecond = rpmToVelocity( righEncoder.getVelocity() );
        return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
    }

    // TODO: get actual distance
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double leftDistanceMeters = leftEncoder.getPosition() / getCurrentGearRatio() * getWheelCircumference() ;
        double rightDistanceMeters = righEncoder.getPosition() / getCurrentGearRatio() * getWheelCircumference() ;
        odometry.update(navx.getRotation2d(), leftDistanceMeters, rightDistanceMeters);

        SmartDashboard.putString("Current Gear", getGear() ? "High Gear":"Low Gear");
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
