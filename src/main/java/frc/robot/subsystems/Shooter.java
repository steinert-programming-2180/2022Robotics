package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public CANSparkMax bottomFlywheel;
  public CANSparkMax topFlywheel;

  public Shooter() {
    bottomFlywheel = new CANSparkMax(ShooterConstants.bottomFlywheelPort, MotorType.kBrushless);
    topFlywheel = new CANSparkMax(ShooterConstants.topFlywheelPort, MotorType.kBrushless);
  }

  public void shoot() {
    bottomFlywheel.set(ShooterConstants.shooterSpeed);
    topFlywheel.set(ShooterConstants.shooterSpeed);
  }

  public void stopShooting() {
    bottomFlywheel.set(0);
    topFlywheel.set(0);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
