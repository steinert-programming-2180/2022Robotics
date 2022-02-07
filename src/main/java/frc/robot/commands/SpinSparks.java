// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Conveyor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** An example command that uses an example subsystem. */
public class SpinSparks extends CommandBase {
  private static final Command SpinSparks = null;
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Conveyor spinsparks;
  Joystick Joy0;
  JoystickButton b1;
  public Spark bottomS;
  public Spark topS;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param shooter The subsystem used by this command.
   */
  public SpinSparks(Conveyor shooter) {
    spinsparks = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    b1 = new JoystickButton(Joy0, 1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
