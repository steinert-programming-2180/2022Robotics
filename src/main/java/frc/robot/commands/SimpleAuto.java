// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class SimpleAuto extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  // ConveyorCommand conveyorCommand, TimedDrive timedDrive
  public SimpleAuto(SetArm raiseArm, ShooterCommand shooterCommand, ConveyorCommand conveyorCommand, TimedDrive timedDrive) {
      TimedCommand timedShooter = new TimedCommand(shooterCommand, AutonomousConstants.shooterTime);
      TimedCommand timedConveyor = new TimedCommand(conveyorCommand, AutonomousConstants.conveyorTime);
      
      addCommands(raiseArm, timedShooter, timedConveyor, timedDrive);
  }
}