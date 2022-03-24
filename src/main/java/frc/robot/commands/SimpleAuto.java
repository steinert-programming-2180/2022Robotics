// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonomousConstants;

/** An example command that uses an example subsystem. */
public class SimpleAuto extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  // ConveyorCommand conveyorCommand, TimedDrive timedDrive
  public SimpleAuto(RaiseArm raiseArm, ShooterCommand shooterCommand, ConveyorCommand conveyorCommand, TimedDrive timedDrive) {
      TimedCommand timedShooter = new TimedCommand(shooterCommand, AutonomousConstants.shooterTime);
      TimedCommand timedConveyor = new TimedCommand(conveyorCommand, AutonomousConstants.conveyorTime);
      
      addCommands(raiseArm, timedShooter, timedConveyor, timedDrive);
  }
}