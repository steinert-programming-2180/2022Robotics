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

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public CANSparkMax bottomFlywheel;
  public RelativeEncoder bottomEncoder;
  public CANSparkMax topFlywheel;
  public RelativeEncoder topEncoder;
  PIDController rpmController;
  SimpleMotorFeedforward ff;
  double goalRPM = ShooterConstants.shooterRPM;

  public Shooter() {
    bottomFlywheel = new CANSparkMax(ShooterConstants.bottomFlywheelPort, MotorType.kBrushless);
    topFlywheel = new CANSparkMax(ShooterConstants.topFlywheelPort, MotorType.kBrushless);
    bottomEncoder = bottomFlywheel.getEncoder();
    topEncoder = topFlywheel.getEncoder();

    rpmController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
    rpmController.setSetpoint(ShooterConstants.shooterRPM);
    ff = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

    SmartDashboard.putNumber("Shooter Speed", ShooterConstants.shooterSpeed);
    SmartDashboard.putNumber("Goal RPM", 2);
    setMotorsToCoast();
  }

  void setMotorsToCoast(){
    bottomFlywheel.setIdleMode(IdleMode.kCoast);
    topFlywheel.setIdleMode(IdleMode.kCoast);
  }

  public void setGoalRPM(double rpm){
    goalRPM = rpm;
  }

  public void shoot() {
    double shooterSpeed = SmartDashboard.getNumber("Shooter Speed", ShooterConstants.shooterSpeed);

    setGoalRPM( SmartDashboard.getNumber("Goal RPM", 0) );

    double calculatedSpeed = ff.calculate(goalRPM);
    bottomFlywheel.set(calculatedSpeed);
    topFlywheel.set(calculatedSpeed);
  }

  public void stopShooting() {
    bottomFlywheel.set(0);
    topFlywheel.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Low RPM", bottomEncoder.getVelocity());
    SmartDashboard.putNumber("High RPM", topEncoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {}
}
