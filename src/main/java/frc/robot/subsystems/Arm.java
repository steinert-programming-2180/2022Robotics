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
import frc.robot.ShuffleboardControl;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    CANSparkMax leftArmRaiser;
    CANSparkMax rightArmRaiser;

    DigitalInput lowerLimitSwitch;

    AnalogInput potentiometer;

    final double maxPotentiometerValue = 2682;
    final double minimumPotentiometerValue = 1340;

    PIDController pidController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

    public Arm() {
        leftArmRaiser = new CANSparkMax(ArmConstants.leftArmRaiserPort, MotorType.kBrushless);
        rightArmRaiser = new CANSparkMax(ArmConstants.rightArmRaiserPort, MotorType.kBrushless);

        setArmToBrake();
        pidController.setSetpoint(minimumPotentiometerValue);
        pidController.setTolerance(ArmConstants.potentiometerTolerance);

        rightArmRaiser.follow(leftArmRaiser, true);
        potentiometer = new AnalogInput(ArmConstants.potentiometerPort);

        lowerLimitSwitch = new DigitalInput(ArmConstants.lowerLimitSwitchPort);

        SmartDashboard.putNumber("Setpoint", pidController.getSetpoint());
    }

    void setArmToBrake(){
        leftArmRaiser.setIdleMode(IdleMode.kBrake);
        rightArmRaiser.setIdleMode(IdleMode.kBrake);
    }

    void setArmToCoast(){
        leftArmRaiser.setIdleMode(IdleMode.kCoast);
        rightArmRaiser.setIdleMode(IdleMode.kCoast);
    }
    
    /**
     * If the setpoint is out of the range [1340, 2682], sets it to max or min
     */
    public void setSetpoint(double setpoint){
        setpoint = Math.max(setpoint, minimumPotentiometerValue);
        setpoint = Math.min(setpoint, maxPotentiometerValue);

        pidController.setSetpoint(setpoint);
    }

    public boolean atSetpoint(){
        return pidController.atSetpoint();
    }

    public void raiseArm(){
        moveArm(ArmConstants.armSpeed);
    }

    public void lowerArm(){
        moveArm(-ArmConstants.armSpeed);
    }

    public void moveArm(double speed){
        boolean isTryingToExceedMaximum = hasReachedUpperLimit() && speed > 0;
        boolean isTryingToExceedMinimum = hasReachedLowerLimit() && speed < 0;

        if(isTryingToExceedMaximum || isTryingToExceedMinimum) stopArm();
        else leftArmRaiser.set(speed);
    }

    public void usePID(){
        double rawPIDValue = pidController.calculate(potentiometer.getValue());
        double adjustedPIDValue = MathUtil.clamp(rawPIDValue, -ArmConstants.maxPIDSpeed, ArmConstants.maxPIDSpeed);
        moveArm(adjustedPIDValue);
    }

    public boolean hasReachedLowerLimit(){
        return !lowerLimitSwitch.get();
    }

    public boolean hasReachedUpperLimit(){
        return potentiometer.getValue() >= maxPotentiometerValue;
    }

    public void stopArm() {
        leftArmRaiser.set(0);
    }

    @Override
    public void periodic() {
        usePID();

        ShuffleboardControl.addToDevelopment("Error", pidController.getPositionError());
        ShuffleboardControl.addToDevelopment("Lower Limit Switch", lowerLimitSwitch.get());
        ShuffleboardControl.addToDevelopment("Pot Value", potentiometer.getValue());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
    public void initialize() {
        double initialPotentiometerValue = potentiometer.getValue();
        setSetpoint(initialPotentiometerValue);
    }
}
