// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    CANSparkMax leftArmRaiser;
    CANSparkMax rightArmRaiser;

    DigitalInput lowerLimitSwitch;
    DigitalInput topLimitSwitch;

    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder; //used;
    //distance of 60

    double referencePoint = 0;

    public Arm() {
        leftArmRaiser = new CANSparkMax(ArmConstants.leftArmRaiserPort, MotorType.kBrushless);
        rightArmRaiser = new CANSparkMax(ArmConstants.rightArmRaiserPort, MotorType.kBrushless);
        rightArmRaiser.follow(leftArmRaiser, true);

        leftEncoder = leftArmRaiser.getEncoder();
        rightEncoder = rightArmRaiser.getEncoder();

        lowerLimitSwitch = new DigitalInput(ArmConstants.lowerLimitSwitchPort);
        //topLimitSwitch = new DigitalInput(ArmConstants.topLimitSwitchPort);
    }

    public void resetReferencePoint(){
        rightEncoder.setPosition(0);
    }

    public void raiseArm(){
        if(hasReachedUpperLimit()){
            stopArm();
            return;
        }
        leftArmRaiser.set(ArmConstants.armSpeed);
    }

    public void lowerArm(){
        // When triggered, lower limit switch is false
        if(hasReachedLowerLimit()) {
            stopArm();
            return;
        }
        leftArmRaiser.set(-ArmConstants.armSpeed);
    }

    public boolean hasReachedLowerLimit(){
        return !lowerLimitSwitch.get();
    }

    public boolean hasReachedUpperLimit(){
        return rightEncoder.getPosition() >= referencePoint + 80;
    }

    public void stopArm() {
        leftArmRaiser.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Lower Limit Switch", lowerLimitSwitch.get());
        SmartDashboard.putNumber("Arm Encoder", rightEncoder.getPosition());

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
    public void initialize() {
    }
}
