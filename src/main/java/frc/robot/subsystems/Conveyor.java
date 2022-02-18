package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Spark entranceConveyor;
  public Spark exitConveyor;

  public Conveyor() {
    entranceConveyor = new Spark(ConveyorConstants.entranceConveyorPort);
    exitConveyor = new Spark(ConveyorConstants.exitConveyorPort);
  }

  public void convey() {
    entranceConveyor.set(ConveyorConstants.conveyorSpeed);
    exitConveyor.set(-ConveyorConstants.conveyorSpeed);
  }

  public void reverseConvey() {
    entranceConveyor.set(-ConveyorConstants.conveyorSpeed);
    exitConveyor.set(ConveyorConstants.conveyorSpeed);
  }

  public void stopConveyor() {
    entranceConveyor.set(0);
    exitConveyor.set(0);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
