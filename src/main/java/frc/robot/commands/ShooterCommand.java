package frc.robot.commands;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Shooter shootercommand;
  
  public ShooterCommand(Shooter shooter) {
    shootercommand = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shootercommand.shoot();
  }

  @Override
  public void execute() {
    shootercommand.shoot();
  }

  @Override
  public void end(boolean interrupted) {
    shootercommand.shootstop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
