package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimedCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private CommandBase command;
    private long startTime;
    double timeInSeconds;
    
    public TimedCommand(CommandBase command, double timeInSeconds) {
      this.command = command;
      this.timeInSeconds = timeInSeconds;
    }
  
    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        command.initialize();
    }
  
    @Override
    public void execute() {
        command.execute();
    }
  
    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }
  
    @Override
    public boolean isFinished() {
      return System.currentTimeMillis() - startTime > timeInSeconds * 1000 || command.isFinished();
    }
}
