package frc.robot.commands;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Shooter shooter;
  
  public ShooterCommand(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.shoot();
  }

  @Override
  public void execute() {
    shooter.shoot();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShooting();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
