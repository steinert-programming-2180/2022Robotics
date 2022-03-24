// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class Turn extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drivetrain drivetrain;

    double kP = DriveConstants.kP;
    double kI = DriveConstants.kI;
    double kD = DriveConstants.kD;
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
