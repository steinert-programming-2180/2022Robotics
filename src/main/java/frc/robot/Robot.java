// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  Joystick leftJoystick, rightJoystick, middleJoystick;
  XboxController xbox;

  DriveTrain driveTrainSubsystem;

  boolean reverse = false;

  boolean arcade = false;
  AHRS navx;
  PIDController turnController;
  SimpleMotorFeedforward sim;

  double speed;
  double angle;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    driveTrainSubsystem = new DriveTrain();

    double kP = 0.02;
    double kI = 0.001; // 0.025
    double kD = 0;
    turnController = new PIDController(kP, kI, kD);
    turnController.setSetpoint(0);

    double kS = 1;
    double kV = 1;
    double kA = 1;
    sim = new SimpleMotorFeedforward(kS, kV, kA);

    SmartDashboard.putNumber("SetP", SmartDashboard.getNumber("SetP", 0.02));
    SmartDashboard.putNumber("SetI", SmartDashboard.getNumber("SetI", 0.001));
    SmartDashboard.putNumber("SetD", SmartDashboard.getNumber("SetD", 0));

    // Create the Joysticks

    leftJoystick = new Joystick(Constants.leftJoystickPort);
    rightJoystick = new Joystick(Constants.rightJoystickPort);
    middleJoystick = new Joystick(3);
    xbox = new XboxController(4);

    navx = new AHRS(Port.kMXP);
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
    angle = navx.getYaw();
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Angle Graph", angle);
    SmartDashboard.putNumber("Setpoint", turnController.getSetpoint());
    SmartDashboard.putNumber("New Force", speed);

    if(leftJoystick.getTriggerPressed()){
      navx.zeroYaw();
    }

    // SmartDashboard.putNumber("P", MathUtil.clamp(leftJoystick.getZ(), 0, 5));
    // SmartDashboard.putNumber("I", MathUtil.clamp(rightJoystick.getZ(), 0, 5));
    // SmartDashboard.putNumber("D", MathUtil.clamp(middleJoystick.getZ(), 0, 5));
    
    SmartDashboard.putNumber("P", turnController.getP());
    SmartDashboard.putNumber("I", turnController.getI());
    SmartDashboard.putNumber("D", turnController.getD());

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
    double leftSpeed = leftJoystick.getY();
    double rightSpeed = rightJoystick.getY();
    // double max = 1.0/180.0;

    double p = SmartDashboard.getNumber("SetP", 1.0/180.0);
    double i = SmartDashboard.getNumber("SetI", 0);
    double d = SmartDashboard.getNumber("SetD", 0);

    SmartDashboard.putNumber("Error", turnController.getPositionError());
    SmartDashboard.putNumber("Error Graph", turnController.getPositionError());


    // double leftClamp = MathUtil.clamp(leftJoystick.getZ(), 0, max);
    // double rightClamp = MathUtil.clamp(rightJoystick.getZ(), 0, 1.0/180 + 1);
    // double middleClamp = MathUtil.clamp(middleJoystick.getZ(), 0, 1);

    turnController.setP(p);
    turnController.setI(i);
    turnController.setD(d);

    boolean isHumanControlled = true;

    if(leftJoystick.getRawButton(2)){
      isHumanControlled = !isHumanControlled;
    }

    if (rightJoystick.getRawButton(1)){
      turnController.setSetpoint(135);
    }

    if (middleJoystick.getTrigger()){
      turnController.setSetpoint(45);
    }

    if(xbox.getYButton()) turnController.setSetpoint(0);
    else if(xbox.getBButton())turnController.setSetpoint(90);
    else if(xbox.getAButton()) turnController.setSetpoint(180);
    else if(xbox.getXButton()) turnController.setSetpoint(-90);

    SmartDashboard.putBoolean("Is Human Controlled", isHumanControlled);

    speed = MathUtil.clamp(turnController.calculate(angle), -1, 1);

    if(isHumanControlled) driveTrainSubsystem.drive(leftSpeed, rightSpeed);
    else driveTrainSubsystem.drive(0, speed);
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
