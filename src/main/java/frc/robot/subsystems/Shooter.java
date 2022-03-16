package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ShooterCommand;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public CANSparkMax bottomFlywheel;
  public RelativeEncoder bottomEncoder;
  public CANSparkMax topFlywheel;
  public RelativeEncoder topEncoder;
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
  double goalRPM = ShooterConstants.shooterRPM;

  public Shooter() {
    bottomFlywheel = new CANSparkMax(ShooterConstants.bottomFlywheelPort, MotorType.kBrushless);
    topFlywheel = new CANSparkMax(ShooterConstants.topFlywheelPort, MotorType.kBrushless);
    bottomEncoder = bottomFlywheel.getEncoder();
    topEncoder = topFlywheel.getEncoder();

    SmartDashboard.putNumber("Goal RPM", goalRPM);
    setMotorsToCoast();
  }

  void setMotorsToCoast(){
    bottomFlywheel.setIdleMode(IdleMode.kCoast);
    topFlywheel.setIdleMode(IdleMode.kCoast);
  }

  public void setGoalRPM(double rpm){ goalRPM = rpm / 60; }

  public void shoot() {
    double calculatedSpeed = feedforward.calculate(goalRPM);

    bottomFlywheel.setVoltage(calculatedSpeed);
    topFlywheel.setVoltage(calculatedSpeed);
  }

  public void stopShooting() {
    bottomFlywheel.set(0);
    topFlywheel.set(0);
  }

  @Override
  public void periodic() {
    setGoalRPM( SmartDashboard.getNumber("Goal RPM", ShooterConstants.shooterRPM) );
    SmartDashboard.putNumber("Low RPM", bottomEncoder.getVelocity());
    SmartDashboard.putNumber("High RPM", topEncoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {}
}
