// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IO;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class DefaultDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain drivetrain;

  Joystick leftJoystick;
  Joystick righJoystick;

  double speedLimit = DriveConstants.initialSpeedLimit;

  public DefaultDrive(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  public void setSpeedLimit(double speedLimit){
    this.speedLimit = speedLimit;
  }
  
  public void resetSpeedLimit(){
    setSpeedLimit(DriveConstants.initialSpeedLimit);
  }

  public void removeSpeedLimit() {
    setSpeedLimit(1);
  }

  @Override
  public void initialize() {
    leftJoystick = new Joystick(IO.leftJoystickPort);
    righJoystick = new Joystick(IO.rightJoystickPort);
  }

  @Override
  public void execute() {
    double leftSpeed = leftJoystick.getY() * speedLimit;
    double rightSpeed = righJoystick.getY() * speedLimit;
    drivetrain.drive(leftSpeed, rightSpeed);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
