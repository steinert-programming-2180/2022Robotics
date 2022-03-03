package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public CANSparkMax entranceConveyor;
  public CANSparkMax exitConveyor;

  public Conveyor() {
    entranceConveyor = new CANSparkMax(ConveyorConstants.entranceConveyorPort, MotorType.kBrushless);
    exitConveyor = new CANSparkMax(ConveyorConstants.exitConveyorPort, MotorType.kBrushless);
  }

  public void convey() {
    entranceConveyor.set(-ConveyorConstants.conveyorSpeed);
    exitConveyor.set(ConveyorConstants.conveyorSpeed);
  }

  public void reverseConvey() {
    entranceConveyor.set(ConveyorConstants.conveyorSpeed);
    exitConveyor.set(-ConveyorConstants.conveyorSpeed);
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
