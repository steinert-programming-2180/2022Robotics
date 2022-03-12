package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public CANSparkMax bottomFlywheel;
  public RelativeEncoder bottomEncoder;
  public CANSparkMax topFlywheel;
  public RelativeEncoder topEncoder;

  public Shooter() {
    bottomFlywheel = new CANSparkMax(ShooterConstants.bottomFlywheelPort, MotorType.kBrushless);
    topFlywheel = new CANSparkMax(ShooterConstants.topFlywheelPort, MotorType.kBrushless);
    bottomEncoder = bottomFlywheel.getEncoder();
    topEncoder = topFlywheel.getEncoder();
    SmartDashboard.putNumber("Shooter Speed", ShooterConstants.shooterSpeed);

  }

  // meters per second
  public void shootAtSpeed(double speed){
    bottomEncoder.getVelocity();
  }

  public void shoot() {
    double shooterSpeed = SmartDashboard.getNumber("Shooter Speed", ShooterConstants.shooterSpeed);
    bottomFlywheel.set(shooterSpeed);
    topFlywheel.set(shooterSpeed);
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
