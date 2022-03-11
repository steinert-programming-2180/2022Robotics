// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.Drive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GyroSubsystem;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Turn extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drivetrain drivetrain;

    double kP = Drive.kP;
    double kI = Drive.kI;
    double kD = Drive.kD;
    PIDController turnController = new PIDController(kP, kI, kD);

    public Turn(Drivetrain drivetrain, int targetAngle) {
        this.drivetrain = drivetrain;
        turnController.setSetpoint(targetAngle);
        turnController.setTolerance(5);
        //turnController.enableContinuousInput(-180, 180);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        drivetrain.setMotorsToCoast();
        double pidCalculation = turnController.calculate(drivetrain.getAngle());
        double leftSpeed = MathUtil.clamp(pidCalculation, -1, 1);
        double rightSpeed = leftSpeed;
        drivetrain.drive(0, rightSpeed);

        SmartDashboard.putNumber("Error", turnController.getPositionError());
        SmartDashboard.putNumber("PID Value", pidCalculation);
        SmartDashboard.putNumber("Setpoint", turnController.getSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0,0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
