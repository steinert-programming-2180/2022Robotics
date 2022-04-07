// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ConveyorConstants.ConveyorSection;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Intake intake;
    private final Conveyor conveyor;

    private final ColorSensorV3 colorSensor;
    private final ColorMatch colorMatcher = new ColorMatch();

    private final Color redTarget = new Color(0.56, 0.32, 0.11);
    private final Color blueTarget = new Color(0.14, 0.38, 0.46);
    private Color ourColor;

    boolean isExitFull;
    boolean isEntranceFull;

    boolean twoBall = true;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public IntakeCommand(Intake intake, Conveyor conveyor) {
        this(intake, conveyor, true);
    }

    public IntakeCommand(Intake intake, Conveyor conveyor, boolean twoBall) {
        this.colorSensor = new ColorSensorV3(Port.kOnboard);
        colorMatcher.addColorMatch(redTarget);
        colorMatcher.addColorMatch(blueTarget);

        this.intake = intake;
        this.conveyor = conveyor;
        this.twoBall = twoBall;

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
        intake.extendIntake();

        if(isOurBall()) intake();
        else outakeEntrance();
    }

    void intake(){
        isExitFull = isExitFull();
        isEntranceFull = isEntranceFull();

        if (isExitFull && isEntranceFull) {
            intake.intakeStop();
            conveyor.stopConveyor();
        } else if (isExitFull && !isEntranceFull) {
            intake.intakeSpin();
            conveyor.convey(ConveyorSection.ENTRANCE);
            conveyor.stopConveyor(ConveyorSection.EXIT);
        } else if (isEntranceFull && !isExitFull) {
            intake.intakeSpin();
            conveyor.convey();
        } else {
            intake.intakeSpin();
            conveyor.convey();
        }
    }

    void outakeEntrance(){
        conveyor.conveyorReverse(ConveyorSection.ENTRANCE);
        intake.intakeReverse();
    }

    boolean isOurBall(){
        Color colorDetected = colorSensor.getColor();
        ColorMatchResult closestMatch = colorMatcher.matchClosestColor(colorDetected);

        if(closestMatch.color != getAllianceColor()) return false;
        else return true;
    }

    Color getAllianceColor(){
        Alliance ourAlliance = DriverStation.getAlliance();

        if(ourAlliance == Alliance.Blue) return blueTarget;
        else return redTarget;
    }

    boolean isExitFull() {
        return conveyor.getBeamBreakStatus(ConveyorSection.EXIT);
    }

    boolean isEntranceFull() {
        return conveyor.getBeamBreakStatus(ConveyorSection.ENTRANCE);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.intakeStop();
        conveyor.stopConveyor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (twoBall)
            return isExitFull() && isEntranceFull();
        return isExitFull();
    }
}
