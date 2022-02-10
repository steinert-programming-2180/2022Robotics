package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCom extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Intake m_Intake;
    
/**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command
    
   */
  public IntakeCom(Intake subsystem) {
    m_Intake = subsystem;
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_Intake.intakeSpin();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_Intake.intakeSpin();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_Intake.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
    

