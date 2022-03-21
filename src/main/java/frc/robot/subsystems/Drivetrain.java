// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ShuffleboardControl;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    AHRS navx;

    DifferentialDriveOdometry odometry;

    MotorControllerGroup leftMotorGroup;
    MotorControllerGroup rightMotorGroup;

    DifferentialDrive drive;

    CANSparkMax[] leftMotors;
    CANSparkMax[] rightMotors;

    boolean inHighGear = true;
    
    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    DoubleSolenoid shifters;

    /** Creates a new ExampleSubsystem. */
    public Drivetrain() {
        setupMotorArrays();
        leftMotorGroup = new MotorControllerGroup(leftMotors);
        rightMotorGroup = new MotorControllerGroup(rightMotors);

        // Invert to drive propery
        // leftMotorGroup.setInverted(true);
        // rightMotorGroup.setInverted(true);

        // Create the Differential Drive to drive the robot
        drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

        navx = new AHRS(Port.kMXP);
        leftEncoder = leftMotors[0].getEncoder();
        rightEncoder = rightMotors[0].getEncoder();
        odometry = new DifferentialDriveOdometry(navx.getRotation2d());
        resetSensors();

        shifters = new DoubleSolenoid(Constants.PneumaticHubPort, PneumaticsModuleType.REVPH, DriveConstants.highGearSolenoid, DriveConstants.lowGearSolenoid);
        inHighGear = getGear();
    }

    public void resetSensors(){
        navx.reset();
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        resetOdometry(new Pose2d());
    }

    public double getAngle(){
        return navx.getYaw();
    }

    // True if high; False if low;
    public boolean getGear(){
        return shifters.get() == Value.kForward;
    }

    public double getCurrentGearRatio(){
        // return getGear() ? DriveConstants.highGearRatio : DriveConstants.lowGearRatio;
        return DriveConstants.lowGearRatio;
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
        for (int i = 0; i < DriveConstants.leftMotorPorts.length; i++){
            leftMotors[i] = new CANSparkMax(DriveConstants.leftMotorPorts[i], MotorType.kBrushless);
            leftMotors[i].setInverted(false);
        }

        // Make Right Sparks from the ports
        for (int i = 0; i < amountOfRightMotors; i++){
            rightMotors[i] = new CANSparkMax(DriveConstants.rightMotorPorts[i], MotorType.kBrushless);
            rightMotors[i].setInverted(true);
        }
    }

    public void drive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed, getGear());
    }

    public void driveByVoltage(double leftVoltage, double rightVoltage) {
        leftMotorGroup.setVoltage(leftVoltage);
        rightMotorGroup.setVoltage(rightVoltage);
        drive.feed();
    }

    public void setMotorsToCoast() {
        for(CANSparkMax i : leftMotors){
            i.setIdleMode(IdleMode.kCoast);
        }

        for(CANSparkMax i : rightMotors){
            i.setIdleMode(IdleMode.kCoast);
        }
    }

    public void setMotorsToBrake() {
        for(CANSparkMax i : leftMotors){
            i.setIdleMode(IdleMode.kBrake);
        }
        
        for(CANSparkMax i : rightMotors){
            i.setIdleMode(IdleMode.kBrake);
        }
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

    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(pose, navx.getRotation2d());
    }
    
    // TODO: get actual meters per second
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        double leftMetersPerSecond = rpmToVelocity( leftEncoder.getVelocity() );
        double rightMetersPerSecond = rpmToVelocity( rightEncoder.getVelocity() );

        return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
    }

    // TODO: get actual distance
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        getWheelSpeeds();
        double leftDistanceMeters = leftEncoder.getPosition() / getCurrentGearRatio() * getWheelCircumference() ;
        double rightDistanceMeters = rightEncoder.getPosition() / getCurrentGearRatio() * getWheelCircumference() ;
        odometry.update(Rotation2d.fromDegrees(navx.getAngle()), leftDistanceMeters, rightDistanceMeters);

        ShuffleboardControl.addToDevelopment("Current Gear", getGear() ? "High Gear":"Low Gear");
        
        SmartDashboard.putBoolean("High Gear?", getGear());

        ShuffleboardControl.addToDevelopment("X", getPose().getX());
        ShuffleboardControl.addToDevelopment("Y", getPose().getY());
    }
}