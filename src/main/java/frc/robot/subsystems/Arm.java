// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.ShuffleboardControl;

public class Arm extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    CANSparkMax leftArmRaiser;
    CANSparkMax rightArmRaiser;
    RelativeEncoder armEncoder;

    DigitalInput lowerLimitSwitch;

    AnalogInput potentiometer;
    double maxEncoderVal = ArmConstants.maxEncoderVal;

    public Arm() {
        leftArmRaiser = new CANSparkMax(ArmConstants.leftArmRaiserPort, MotorType.kBrushless);
        rightArmRaiser = new CANSparkMax(ArmConstants.rightArmRaiserPort, MotorType.kBrushless);
        rightArmRaiser.follow(leftArmRaiser, true);

        armEncoder = rightArmRaiser.getEncoder();
        setArmToCoast();

        SmartDashboard.putNumber("Max Encoder Val", maxEncoderVal);

        potentiometer = new AnalogInput(ArmConstants.potentiometerPort);

        lowerLimitSwitch = new DigitalInput(ArmConstants.lowerLimitSwitchPort);
    }

    public void initialize(){}

    void setArmToBrake(){
        leftArmRaiser.setIdleMode(IdleMode.kBrake);
        rightArmRaiser.setIdleMode(IdleMode.kBrake);
    }

    void setArmToCoast(){
        leftArmRaiser.setIdleMode(IdleMode.kCoast);
        rightArmRaiser.setIdleMode(IdleMode.kCoast);
    }

    public void raiseArm(){
        moveArm(ArmConstants.armSpeed);
    }

    public void lowerArm(){
        moveArm(-0.4);
    }

    public void moveArm(double speed){
        boolean isTryingToExceedMaximum = hasReachedUpperLimit() && speed > 0;
        boolean isTryingToExceedMinimum = hasReachedLowerLimit() && speed < 0;

        if(isTryingToExceedMaximum || isTryingToExceedMinimum) stopArm();
        else leftArmRaiser.set(speed);
    }

    public boolean hasReachedLowerLimit(){
        return !lowerLimitSwitch.get();
    }

    public boolean hasReachedUpperLimit(){
        return getEncoderValue() >= maxEncoderVal;
    }

    public double getEncoderValue(){
        return armEncoder.getPosition();
    }

    public void stopArm() {
        leftArmRaiser.set(0);
    }

    @Override
    public void periodic() {
        if(hasReachedLowerLimit()) armEncoder.setPosition(0);

        maxEncoderVal = SmartDashboard.getNumber("Max Encoder Val", 60);
        ShuffleboardControl.addToDevelopment("Lower Limit Switch", lowerLimitSwitch.get());
        ShuffleboardControl.addToDevelopment("Arm Encoder", armEncoder.getPosition());
        ShuffleboardControl.addToDevelopment("Pot Value", potentiometer.getValue());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
