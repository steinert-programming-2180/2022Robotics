package frc.robot.commands;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlledShooter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Shooter shooter;
  private Conveyor conveyor;
  long startTime;
  double delay;
  
  public ControlledShooter(Shooter shooter, Conveyor conveyor, double delay) {
    this.shooter = shooter;
    this.conveyor = conveyor;
    this.delay = delay;

    addRequirements(shooter);
    addRequirements(conveyor);
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    shooter.shoot();
  }

  @Override
  public void execute() {
    shooter.shoot();
    if(System.currentTimeMillis() - startTime >= delay)
      conveyor.convey();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShooting();
    conveyor.stopConveyor();
  }

  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - startTime <= delay + 1000;
  }
}
