package frc.robot.commands;

import frc.robot.subsystems.Conveyor;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConveyorBCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Conveyor conveyorbcommand;
  
  public ConveyorBCommand(Conveyor conveyor) {
    conveyorbcommand = conveyor;
    addRequirements(conveyor);
  }

  @Override
  public void initialize() {
    conveyorbcommand.conveyreverse();
  }

  @Override
  public void execute() {
    conveyorbcommand.conveyreverse();
  }

  @Override
  public void end(boolean interrupted) {
    conveyorbcommand.conveystop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
