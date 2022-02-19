// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IO;
import frc.robot.commands.ConveyorBackwardCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TakeAndShoot;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake intake = new Intake();
  private final Conveyor conveyor = new Conveyor();
  private final Shooter shooter = new Shooter();
  private final TakeAndShoot takeAndShoot = new TakeAndShoot(intake, conveyor, shooter);

  private final IntakeCommand intakeCommand = new IntakeCommand(intake);
  private final IntakeReverse intakeReverse = new IntakeReverse(intake);
  private final ConveyorCommand conveyorCommand = new ConveyorCommand(conveyor);
  private final ConveyorBackwardCommand conveyorBackwardCommand = new ConveyorBackwardCommand(conveyor);
  private final ShooterCommand shooterCommand = new ShooterCommand(shooter);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button 
    configureButtonBindings();
    XboxController Xbox = new XboxController(IO.xboxPort);
    JoystickButton aButton = new JoystickButton(Xbox, 1);
    JoystickButton bButton = new JoystickButton(Xbox, 2);
    JoystickButton xButton = new JoystickButton(Xbox, 3);
    JoystickButton yButton = new JoystickButton(Xbox, 4);
    JoystickButton lButton = new JoystickButton(Xbox, 5);
    JoystickButton rButton = new JoystickButton(Xbox, 6);
    JoystickButton backButton = new JoystickButton(Xbox, 7);
    JoystickButton startButton = new JoystickButton(Xbox, 8);
    JoystickButton lStick = new JoystickButton(Xbox, 9);

    aButton.whenHeld(takeAndShoot);
    bButton.whenHeld(intakeCommand).whenHeld(conveyorCommand);
    xButton.whenHeld(intakeReverse).whenHeld(conveyorBackwardCommand);
    yButton.whenHeld(conveyorCommand).whenHeld(shooterCommand);
    lButton.whenHeld(conveyorBackwardCommand);
    rButton.whenHeld(conveyorCommand);
    backButton.whenHeld(intakeReverse);
    startButton.whenHeld(intakeCommand);
    lStick.whenHeld(shooterCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
