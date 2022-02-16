package frc.robot.commands;

import frc.robot.subsystems.Conveyor;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConveyorCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Conveyor conveyorcommand;
  
  public ConveyorCommand(Conveyor conveyor) {
    conveyorcommand = conveyor;
    addRequirements(conveyor);
  }

  @Override
  public void initialize() {
    conveyorcommand.convey();
  }

  @Override
  public void execute() {
    conveyorcommand.convey();
  }

  @Override
  public void end(boolean interrupted) {
    conveyorcommand.conveystop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
