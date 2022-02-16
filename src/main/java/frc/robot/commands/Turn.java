// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GyroSubsystem;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Turn extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveTrain drivetrain;
    private final GyroSubsystem gyroSubsystem;

    double kP = Drive.kP;
    double kI = Drive.kI;
    double kD = Drive.kD;
    PIDController turnController = new PIDController(kP, kI, kD);

    public Turn(DriveTrain drivetrain, GyroSubsystem gyroSubsystem, int targetAngle) {
        this.gyroSubsystem = gyroSubsystem;
        this.drivetrain = drivetrain;
        turnController.setSetpoint(targetAngle);

        addRequirements(drivetrain);
        addRequirements(gyroSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double pidCalculation = turnController.calculate(gyroSubsystem.getAngle());
        double leftSpeed = 0;
        double rightSpeed = MathUtil.clamp(pidCalculation, -1, 1);
        drivetrain.drive(leftSpeed, rightSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0,0);
    }

    @Override
    public boolean isFinished() {
        return turnController.atSetpoint();
    }
}
