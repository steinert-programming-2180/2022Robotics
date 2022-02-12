// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.commands.limelight.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Limelight m_limelight = new Limelight();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final LimelightPeriodic m_limeperiodic = new LimelightPeriodic(m_limelight);

  private final ParallelRaceGroup m_pipelinecam = new PipelineCamera(m_limelight).withTimeout(0.01);
  private final ParallelRaceGroup m_drivecam = new DriveCamera(m_limelight).withTimeout(0.01);

  private final ParallelRaceGroup m_disable = new DisableLights().withTimeout(0.01);
  private final ParallelRaceGroup m_enable = new EnableLights().withTimeout(0.01);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_limelight.setDefaultCommand(m_limeperiodic);
    SmartDashboard.putData("Drive Camera", m_drivecam);
    SmartDashboard.putData("Pipeline Cam", m_pipelinecam);
    SmartDashboard.putData("Enable Lights", m_enable);
    SmartDashboard.putData("Disable Lights", m_disable);
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
    return m_autoCommand;
  }

  	/**
	 * Initialize value on SmartDashboard for user input, but leave old value if
	 * already present.
	 *
	 * @param key      The SmartDashboard key to associate with the value.
	 * @param defValue The default value to assign if not already on dashboard.
	 *
	 * @return The current value that appears on the dashboard.
	 */
	public static double createSmartDashboardNumber(String key, double defValue) {

		// See if already on dashboard, and if so, fetch current value
		double value = SmartDashboard.getNumber(key, defValue);

		// Make sure value is on dashboard, puts back current value if already set
		// otherwise puts back default value
		SmartDashboard.putNumber(key, value);

		return value;
	}
}
