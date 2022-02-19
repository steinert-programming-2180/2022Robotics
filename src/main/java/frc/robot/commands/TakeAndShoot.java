// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TakeAndShoot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  
  private final Intake intake;
  private final Conveyor conveyor;
  private final Shooter shooter;

  private IntakeCommand intakeCommand;
  private ConveyorCommand conveyorCommand;
  private ShooterCommand shooterCommand;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TakeAndShoot(Intake intake, Conveyor conveyor, Shooter shooter) {
    this.intake = intake;
    this.conveyor = conveyor;
    this.shooter = shooter;

    intakeCommand = new IntakeCommand(intake);
    conveyorCommand = new ConveyorCommand(conveyor);
    shooterCommand = new ShooterCommand(shooter);

    addRequirements(intake);
    addRequirements(conveyor);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      intakeCommand.initialize();
      conveyorCommand.initialize();
      shooterCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      intakeCommand.execute();
      conveyorCommand.execute();
      shooterCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      intakeCommand.end(interrupted);
      conveyorCommand.end(interrupted);
      shooterCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
