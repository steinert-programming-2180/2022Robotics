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
import frc.robot.commands.SimpleAuto;
import frc.robot.commands.ConveyorBackwardCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TimedDrive;
import frc.robot.commands.LowerArm;
import frc.robot.commands.RaiseArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
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

  private final LowerArm lowerArm = new LowerArm(arm);
  private final RaiseArm raiseArm = new RaiseArm(arm);

  private final TimedDrive timedDrive = new TimedDrive(drivetrain, -DriveConstants.autonomousSpeed, AutonomousConstants.driveTime);
  

  // Emergency autonomous. not actual autonomous unfortunately
  private SimpleAuto simpleAuto = new SimpleAuto(new RaiseArm(arm), shooterCommand, conveyorCommand, timedDrive);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button
    configureButtonBindings();

    drivetrain.setDefaultCommand(driveCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Joystick leftJoystick = new Joystick(IO.leftJoystickPort);
    Joystick rightJoystick = new Joystick(IO.rightJoystickPort);
    XboxController xbox = new XboxController(IO.xboxPort);

    JoystickButton lowGearButton = new JoystickButton(leftJoystick, 3);
    JoystickButton highGearButton = new JoystickButton(rightJoystick, 3);
    JoystickButton leftTrigger = new JoystickButton(leftJoystick, Joystick.ButtonType.kTrigger.value);
    JoystickButton rightTrigger = new JoystickButton(rightJoystick, Joystick.ButtonType.kTrigger.value);

    JoystickButton aButton = new JoystickButton(xbox, 1);
    JoystickButton bButton = new JoystickButton(xbox, 2);
    JoystickButton xButton = new JoystickButton(xbox, 3);
    JoystickButton yButton = new JoystickButton(xbox, 4);
    JoystickButton leftBumper = new JoystickButton(xbox, 5);
    JoystickButton rightBumper = new JoystickButton(xbox, 6);
    JoystickButton backButton = new JoystickButton(xbox, 7);
    JoystickButton startButton = new JoystickButton(xbox, 8);
    JoystickButton leftStick = new JoystickButton(xbox, 9);
    JoystickButton rightStick = new JoystickButton(xbox, 10);

    // Operator
    aButton.whileHeld(intakeCommand);
    xButton.whileHeld(shooterCommand);
    bButton.whileHeld(conveyorCommand);
    yButton.whenPressed(() -> intake.extendOrRetract());

    startButton.whenPressed(raiseArm).whenPressed(shooterCommand);
    backButton.whenPressed(lowerArm).cancelWhenPressed(shooterCommand);

    leftStick.whenPressed(intakeReverse);
    rightStick.whenPressed(conveyorBackwardCommand);

    leftBumper.whileHeld(() -> arm.lowerArm()).whenReleased(() -> arm.stopArm()).cancelWhenPressed(raiseArm).cancelWhenPressed(lowerArm);
    rightBumper.whileHeld(() -> arm.raiseArm()).whenReleased(() -> arm.stopArm()).cancelWhenPressed(raiseArm).cancelWhenPressed(lowerArm);

    // Driver:
    highGearButton.whenPressed(() -> drivetrain.highGear());
    lowGearButton.whenPressed(() -> drivetrain.lowGear());
    leftTrigger.or(rightTrigger).whenActive(() -> driveCommand.setSpeedLimit(DriveConstants.secondSpeedLimit)).whenInactive(() -> driveCommand.resetSpeedLimit());
    leftTrigger.and(rightTrigger).whenActive(() -> driveCommand.removeSpeedLimit()).whenInactive(() -> driveCommand.resetSpeedLimit());
  }

  public void turnOffEverything(){
    intake.intakeStop();
    conveyor.stopConveyor();
    shooter.stopShooting();
    arm.stopArm();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    CommandBase autonomousCommand = emptyCommand;

    int autonomousMode = ShuffleboardControl.getAutonomousMode();

    // These are the indexes of each auto mode in the array (refer to Constants.java)
    switch (autonomousMode) {
      case 0: // None
        autonomousCommand = emptyCommand;
        break;
      case 1: // Drive Backward
        autonomousCommand = timedDrive;
        break;
      case 2: // Simple Auto
        autonomousCommand = simpleAuto;
        break;
      case 3: // Reserved for the paths (NOT WORKING):
        autonomousCommand = emptyCommand; // really, it's getRamseteCommand()
        break;
      default:
        autonomousCommand = emptyCommand;
        break;
    }
    
    return new SequentialCommandGroup(lowerArm, simpleAuto);
  }

  private CommandBase getRamseteCommand(){
    DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);

    Pose2d start = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    List<Translation2d> waypoints = List.of(
        new Translation2d(0, 3));
    Pose2d end = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(3, 3);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        start,
        waypoints,
        end,
        trajectoryConfig);

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
        drivetrain);
        
    return ramseteCommand.andThen(() -> drivetrain.drive(0, 0));
  }
}
