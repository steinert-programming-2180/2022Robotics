package frc.robot.commands;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Shooter shooter;
  double goalRPM = ShooterConstants.atGoalRPM;

  public ShooterCommand(Shooter shooter){
    this(shooter, ShooterConstants.atGoalRPM);
  }
  
  public ShooterCommand(Shooter shooter, double goalRPM) {
    this.shooter = shooter;
    this.goalRPM = goalRPM;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.shoot(goalRPM);
  }

  @Override
  public void execute() {
    shooter.shoot(goalRPM);
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
