// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
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
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Drive;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.Turn;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
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
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem emptySubsystem = new ExampleSubsystem();
  private final ExampleCommand emptyCommand = new ExampleCommand(emptySubsystem);

  private final Drivetrain drivetrain = new Drivetrain();
  private final DefaultDrive driveCommand = new DefaultDrive(drivetrain);
  private final Turn turnCommand = new Turn(drivetrain, 90);

  Joystick leftJoystick = new Joystick(Constants.leftJoystickPort);
  Joystick righJoystick = new Joystick(Constants.rightJoystickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

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
    JoystickButton a = new JoystickButton(leftJoystick, 1);
    a.whenActive(() -> drivetrain.resetAngle());
  }

  public void setDrivetrainMotorsToBrake(){
    drivetrain.setMotorsToBrake();
  }

  public void setDrivetrainMotorsToCoast(){
    drivetrain.setMotorsToCoast();
  }

  public Trajectory openAutoPath(){
    String trajectoryJSON = "paths/FarthestBlue.wpilib.json";
    Trajectory trajectory = null;


    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      System.out.println("Success");
    } catch (IOException ex) {
      System.out.println("Unable to open trajectory: " + trajectoryJSON);
    }

    return trajectory;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    drivetrain.resetSensors();
    DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Drive.trackWidth);

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(3, 2);
    TrajectoryConfig backwardConfig = new TrajectoryConfig(3, 2);
    backwardConfig.setReversed(false);
    trajectoryConfig.setReversed(true);

    Trajectory goToBall = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      List.of(
        new Translation2d(-1, 0)
      ),
      new Pose2d(-2, 0, Rotation2d.fromDegrees(0)),
      trajectoryConfig
    );

    Trajectory goBackToGoal = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      List.of(
        new Translation2d(1, 0)
      ),
      new Pose2d(3, -0.6, Rotation2d.fromDegrees(-45)),
      backwardConfig
    );

    Trajectory goBackToGoal2 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      List.of(
        new Translation2d(1, 0)
      ),
      new Pose2d(3, 0.4, Rotation2d.fromDegrees(45)), 
      backwardConfig
    );

    Trajectory trajectory = goBackToGoal;

    drivetrain.resetOdometry(trajectory.getInitialPose());

    RamseteController ramseteController = new RamseteController(Drive.b, Drive.zeta);

    // https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/creating-following-trajectory.html?highlight=ramsetecommand#creating-the-ramsetecommand 
    RamseteCommand followBallPath = new FollowTrajectory(goToBall, drivetrain);
    RamseteCommand followGoalLeftPath = new FollowTrajectory(goBackToGoal, drivetrain);
    RamseteCommand followGoalRightPath = new FollowTrajectory(goBackToGoal2, drivetrain);

    return followBallPath.andThen(() -> drivetrain.resetSensors()).andThen(followGoalRightPath);
  }
}
