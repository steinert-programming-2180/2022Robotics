// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private ExampleSubsystem emptySubsystem = new ExampleSubsystem();
  private ExampleCommand emptyCommand = new ExampleCommand(emptySubsystem);

  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final DefaultDrive driveCommand = new DefaultDrive(drivetrain);

  private final Intake intake = new Intake();
  private final Conveyor conveyor = new Conveyor();
  private final Shooter shooter = new Shooter();
  private final Arm arm = new Arm();

  private final IntakeCommand intakeCommand = new IntakeCommand(intake, conveyor);
  private final IntakeReverse intakeReverse = new IntakeReverse(intake);
  private final ConveyorCommand conveyorCommand = new ConveyorCommand(conveyor);
  private final ConveyorBackwardCommand conveyorBackwardCommand = new ConveyorBackwardCommand(conveyor);
  private final ShooterCommand shooterCommand = new ShooterCommand(shooter);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button 
    configureButtonBindings();
    Joystick leftJoystick = new Joystick(IO.leftJoystickPort);
    Joystick rightJoystick = new Joystick(IO.rightJoystickPort);

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
    JoystickButton rStick = new JoystickButton(Xbox, 10);
    JoystickButton highGearButton = new JoystickButton(leftJoystick, 3);
    JoystickButton lowGearButton = new JoystickButton(rightJoystick, 3);
    

    // aButton.whenHeld(takeAndShoot);
    aButton.whileHeld(() -> arm.raiseArm()).whenReleased(() -> arm.stopArm());
    //bButton.whenHeld(intakeCommand).whenHeld(conveyorCommand);
    bButton.whileHeld(() -> arm.lowerArm()).whenReleased(() -> arm.stopArm());
    xButton.whenHeld(intakeReverse).whenHeld(conveyorBackwardCommand);
    yButton.whenHeld(conveyorCommand).whenHeld(shooterCommand);
    lButton.whenHeld(conveyorBackwardCommand);
    rButton.whenHeld(conveyorCommand);
    backButton.whenHeld(intakeReverse);
    startButton.whenHeld(intakeCommand);

    highGearButton.whenPressed(() -> drivetrain.highGear());
    lowGearButton.whenPressed(() -> drivetrain.lowGear());

    lStick.whenPressed(() -> intake.extendIntake());
    rStick.whenPressed(() -> intake.retracktIntake());


    // make sure always driving
    drivetrain.setDefaultCommand(driveCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);

    Pose2d start = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    List<Translation2d> waypoints = List.of(
      new Translation2d(0, 3)
    );
    Pose2d end = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(3, 3);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      start,
      waypoints,
      end,
      trajectoryConfig
    );

    RamseteController ramseteController = new RamseteController(DriveConstants.b, DriveConstants.zeta);

    // https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/creating-following-trajectory.html?highlight=ramsetecommand#creating-the-ramsetecommand 
    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory, 
      drivetrain::getPose, 
      ramseteController, 
      new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA),
      kDriveKinematics, 
      drivetrain::getWheelSpeeds, 
      new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD), 
      new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD), 
      drivetrain::driveByVoltage, 
      drivetrain
    );

    return ramseteCommand.andThen(() -> drivetrain.drive(0, 0));
    //return emptyCommand;
  }
}
