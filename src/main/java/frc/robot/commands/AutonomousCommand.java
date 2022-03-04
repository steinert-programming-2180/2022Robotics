// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutonomousCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Drivetrain drivetrain;
  long startTime;

  public AutonomousCommand(Drivetrain drivetrain) {
      this.drivetrain = drivetrain;
      addRequirements(drivetrain);

      startTime = System.currentTimeMillis();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      drivetrain.drive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      drivetrain.drive(DriveConstants.autonomousSpeed, DriveConstants.autonomousSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - startTime >= DriveConstants.autonomousDriveTime * 1000 ;
  }
}
