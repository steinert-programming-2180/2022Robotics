package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Spark bottomFlywheel;
  public Spark topFlywheel;

  public Shooter() {
    bottomFlywheel = new Spark(ShooterConstants.bottomFlywheelPort);
    topFlywheel = new Spark(ShooterConstants.topFlywheelPort);
  }

  public void shoot() {
    bottomFlywheel.set(ShooterConstants.shooterSpeed);
    topFlywheel.set(-ShooterConstants.shooterSpeed);
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
