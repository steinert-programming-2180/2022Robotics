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
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
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
    RelativeEncoder righEncoder;

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
        righEncoder = rightMotors[0].getEncoder();
        odometry = new DifferentialDriveOdometry(navx.getRotation2d());
        resetSensors();

        shifters = new DoubleSolenoid(Constants.PneumaticHubPort, PneumaticsModuleType.REVPH, Drive.highGearSolenoid, Drive.lowGearSolenoid);
        inHighGear = getGear();
    }

    public void resetSensors(){
        navx.reset();
        leftEncoder.setPosition(0);
        righEncoder.setPosition(0);
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
       // return getGear() ? Drive.highGearRatio : Drive.lowGearRatio;
       return Drive.highGearRatio;
    }

    public double rpmToVelocity(double rpm){
        return rpm / getCurrentGearRatio() * (1.0/60.0);
    }

    public double getWheelCircumference(){
        return Drive.wheelDiameter * Math.PI;
    }

    private void setupMotorArrays() {
        int amountOfLeftMotors = Drive.leftMotorPorts.length;
        int amountOfRightMotors = Drive.rightMotorPorts.length;

        leftMotors = new CANSparkMax[amountOfLeftMotors];
        rightMotors = new CANSparkMax[amountOfRightMotors];

        // Make Left Sparks from the ports
        for (int i = 0; i < amountOfLeftMotors; i++){
            leftMotors[i] = new CANSparkMax(Drive.leftMotorPorts[i], MotorType.kBrushless);
            leftMotors[i].setInverted(false);
        }

        // Make Right Sparks from the ports
        for (int i = 0; i < amountOfRightMotors; i++){
            rightMotors[i] = new CANSparkMax(Drive.rightMotorPorts[i], MotorType.kBrushless);
            rightMotors[i].setInverted(true);
        }
    }

    public void drive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
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
        double rightMetersPerSecond = rpmToVelocity( righEncoder.getVelocity() );

        SmartDashboard.putNumber("Left Speed", leftMetersPerSecond);
        SmartDashboard.putNumber("Right Speed", rightMetersPerSecond);
        return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
    }

    // TODO: get actual distance
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        getWheelSpeeds();
        double leftDistanceMeters = leftEncoder.getPosition() / getCurrentGearRatio() * getWheelCircumference() ;
        double rightDistanceMeters = righEncoder.getPosition() / getCurrentGearRatio() * getWheelCircumference() ;
        odometry.update(Rotation2d.fromDegrees(navx.getAngle()), leftDistanceMeters, rightDistanceMeters);

        SmartDashboard.putString("Current Gear", getGear() ? "High Gear":"Low Gear");
        SmartDashboard.putNumber("Angle Accumulation", navx.getRotation2d().getDegrees());
        SmartDashboard.putNumber("Angle", getAngle());
        SmartDashboard.putNumber("Left Distance Meters", leftDistanceMeters);
        SmartDashboard.putNumber("Right Distance Meters", rightDistanceMeters);

        SmartDashboard.putNumber("X", getPose().getX());
        SmartDashboard.putNumber("Y", getPose().getY());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}