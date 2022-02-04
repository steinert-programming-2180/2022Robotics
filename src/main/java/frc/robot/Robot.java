// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  Joystick leftJoystick, rightJoystick;

  DifferentialDrive drive;

  boolean reverse = false;

  boolean arcade = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create the Joysticks

    leftJoystick = new Joystick(Constants.leftJoystickPort);
    rightJoystick = new Joystick(Constants.rightJoystickPort);

    // Create arrays for the left and right Talon sets

    int amountOfLeftMotors = Constants.Drive.leftMotorPorts.length;
    int amountOfRightMotors = Constants.Drive.rightMotorPorts.length;
    WPI_TalonSRX[] leftMotors = new WPI_TalonSRX[amountOfLeftMotors];
    WPI_TalonSRX[] rightMotors = new WPI_TalonSRX[amountOfRightMotors];

    // Make Left Talons from the ports
    for(int i=0; i < amountOfLeftMotors; i++) leftMotors[i] = new WPI_TalonSRX(Constants.Drive.leftMotorPorts[i]);
    
    // Make Right Talons from the ports
    for(int i=0; i < amountOfRightMotors; i++) rightMotors[i] = new WPI_TalonSRX(Constants.Drive.rightMotorPorts[i]);

    // Put motors into their own groups
    MotorControllerGroup leftMotorGroup = new MotorControllerGroup(leftMotors);
    MotorControllerGroup rightMotorGroup = new MotorControllerGroup(rightMotors);
    
    // Invert to drive properly
    rightMotorGroup.setInverted(true);

    // Create the Differential Drive to drive the robot
    drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);


  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // TODO Get actually good drive train code that isn't this

    // Get left and right triggers
    boolean leftTrigger = leftJoystick.getRawButton(1);
    boolean rightTrigger = rightJoystick.getRawButton(1);

    // Create our modifier
    double modifier = Constants.Drive.lowModifier;

    // If both triggers are held, full speed
    if(leftTrigger && rightTrigger) modifier = Constants.Drive.highModifier;

    // Else, if exactly one is held, med speed
    else if (leftTrigger || rightTrigger) modifier = Constants.Drive.medModifier;

    // Else, don't change

    // Calculating leftSpeed and rightSpeed
    double leftSpeed = -leftJoystick.getY() * modifier;
    double rightSpeed = -rightJoystick.getY() * modifier;

    // Getting left and right button 3's if pressed
    boolean leftButton3 = leftJoystick.getRawButtonPressed(3);
    boolean rightButton3 = rightJoystick.getRawButtonPressed(3);

    // If pressed, switch reverse
    if(leftButton3 || rightButton3) {
      reverse = !reverse;
    }

    // Setting these speeds. ? works to change the direction of the robot
    // Last parameter prevents squaring of inputs
    drive.tankDrive(
      !reverse ? leftSpeed : -rightSpeed, 
      !reverse ? rightSpeed : -leftSpeed, 
      false
    );
    

    // Putting leftSpeed and rightSpeed to SmartDashboard
    SmartDashboard.putNumber("Left Speed", leftSpeed);
    SmartDashboard.putNumber("Right Speed", rightSpeed);

    // Putting reverse to SmartDashboard
    SmartDashboard.putBoolean("Forward", !reverse);

    // Temporary BS code to drive straight
    if(leftJoystick.getRawButton(7)) {
      drive.tankDrive(
        !reverse? 1 : -1, 
        !reverse? 1 : -1, 
        false);
    }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
