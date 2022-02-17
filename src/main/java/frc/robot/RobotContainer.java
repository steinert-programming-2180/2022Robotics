// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  DriveTrain driveTrain = new DriveTrain();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
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
    DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Drive.trackWidth);

    Pose2d start = new Pose2d(0, 0, Rotation2d.fromDegrees(45));
    List<Translation2d> waypoints = List.of(
      new Translation2d(0, 3)
    );
    Pose2d end = new Pose2d(3, 0, Rotation2d.fromDegrees(0));
    
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(3, 3);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      start,
      waypoints,
      end,
      trajectoryConfig
    );

    RamseteController ramseteController = new RamseteController();

    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory, 
      driveTrain::getPose, 
      ramseteController, 
      kDriveKinematics, 
      driveTrain::driveByVoltage, 
      driveTrain
    );

    return ramseteCommand.andThen(() -> driveTrain.drive(0, 0));
  }
}
