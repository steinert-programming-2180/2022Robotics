// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ConveyorConstants.ConveyorSection;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Intake intake;
    private final Conveyor conveyor;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public IntakeCommand(Intake intake, Conveyor conveyor) {
        this.intake = intake;
        this.conveyor = conveyor;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);
        addRequirements(conveyor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean isExitFull = conveyor.getBeamBreakStatus(ConveyorSection.EXIT);
        boolean isEntranceFull = conveyor.getBeamBreakStatus(ConveyorSection.ENTRANCE);

        if (isExitFull && isEntranceFull)
            return;

        intake.intakeSpin();
        if (isExitFull)
            conveyor.convey(ConveyorSection.ENTRANCE);
        else
            conveyor.convey();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        conveyor.stopConveyor();
        intake.intakeStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
