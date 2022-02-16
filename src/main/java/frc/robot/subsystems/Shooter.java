package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  public Spark bottomS;
  public Spark topS;

  public Shooter() {
    bottomS = new Spark(0);
    topS = new Spark(1);
  }

  public void shoot() {
    bottomS.set(1);
    topS.set(-1);
  }

  public void shootstop() {
    bottomS.set(0);
    topS.set(0);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
