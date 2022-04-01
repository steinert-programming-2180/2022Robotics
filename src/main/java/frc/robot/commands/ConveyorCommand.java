package frc.robot.commands;

import frc.robot.Constants.ConveyorConstants.ConveyorSection;
import frc.robot.subsystems.Conveyor;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConveyorCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Conveyor conveyor;
  private double startTime;
  
  public ConveyorCommand(Conveyor conveyor) {
    this.conveyor = conveyor;
    addRequirements(conveyor);
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    // Delays 300 milliseconds so balls don't bounce
    if(System.currentTimeMillis() - startTime < 500) {
      conveyor.convey(ConveyorSection.EXIT);
    } else {
      conveyor.convey();
    }
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
