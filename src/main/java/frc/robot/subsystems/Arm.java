// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShuffleboardControl;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    CANSparkMax leftArmRaiser;
    CANSparkMax rightArmRaiser;

    DigitalInput lowerLimitSwitch;

    AnalogInput potentiometer;

    final double goalPotentiometerValue = 2515 - 100;
    final double maxPotentiometerValue = 4096;
    final double minPotentiometerValue = 1294;
    final double maxPotentiometerAngle = 225;

    RelativeEncoder armEncoder; // uses encoder of right motor

    public Arm() {
        leftArmRaiser = new CANSparkMax(ArmConstants.leftArmRaiserPort, MotorType.kBrushless);
        rightArmRaiser = new CANSparkMax(ArmConstants.rightArmRaiserPort, MotorType.kBrushless);

        setArmToBrake();

        rightArmRaiser.follow(leftArmRaiser, true);
        potentiometer = new AnalogInput(ArmConstants.potentiometerPort);

        armEncoder = rightArmRaiser.getEncoder();

        lowerLimitSwitch = new DigitalInput(ArmConstants.lowerLimitSwitchPort);
    }

    double getAngleFromPotentiometer(){
        double rawPotentiometerValue = potentiometer.getValue();
        double potentiometerValueAsPercent = rawPotentiometerValue / maxPotentiometerValue ;
        return potentiometerValueAsPercent * maxPotentiometerAngle;
    }

    void setArmToBrake(){
        leftArmRaiser.setIdleMode(IdleMode.kBrake);
        rightArmRaiser.setIdleMode(IdleMode.kBrake);
    }

    void setArmToCoast(){
        leftArmRaiser.setIdleMode(IdleMode.kCoast);
        rightArmRaiser.setIdleMode(IdleMode.kCoast);
    }

    public void resetReferencePoint(){
        armEncoder.setPosition(0);
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
            resetReferencePoint();
            return;
        }
        leftArmRaiser.set(-ArmConstants.armSpeed);
    }

    public boolean hasReachedLowerLimit(){
        return !lowerLimitSwitch.get();
        // return potentiometer.getValue() <= 1310;
    }

    public boolean hasReachedUpperLimit(){
        // return armEncoder.getPosition() >= 85;
        return potentiometer.getValue() >= goalPotentiometerValue;
    }

    public void stopArm() {
        leftArmRaiser.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        ShuffleboardControl.addToDevelopment("Lower Limit Switch", lowerLimitSwitch.get());
        ShuffleboardControl.addToDevelopment("Arm Encoder", armEncoder.getPosition());
        ShuffleboardControl.addToDevelopment("Pot Value", potentiometer.getValue());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
    public void initialize() {
    }
}
