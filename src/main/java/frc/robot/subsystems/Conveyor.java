package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
  public Spark bottomC;
  public Spark topC;

  public Conveyor() {
    bottomC = new Spark(0);
    topC = new Spark(1);
  }

  public void convey() {
    bottomC.set(1);
    topC.set(-1);
  }

  public void conveyreverse() {
    bottomC.set(-1);
    topC.set(1);
  }

  public void conveystop() {
    bottomC.set(0);
    topC.set(0);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
