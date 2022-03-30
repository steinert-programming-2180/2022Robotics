package frc.robot.commands;

import frc.robot.subsystems.Conveyor;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConveyorReverse extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Conveyor conveyor;
  
  public ConveyorReverse(Conveyor conveyor) {
    this.conveyor = conveyor;
    addRequirements(conveyor);
  }

  @Override
  public void initialize() {
    conveyor.conveyorReverse();
  }

  @Override
  public void execute() {
    conveyor.conveyorReverse();
  }

  @Override
  public void end(boolean interrupted) {
    conveyor.stopConveyor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
