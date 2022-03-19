// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SetArm extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Arm arm;
    private final double targetAngle;

    public SetArm(Arm arm){
        this(arm, ArmConstants.goalPotentiometerValue);
    }

    /**
     * Note: targetAngle is the target value of the potientometer, NOT the actual angle.
     */
    public SetArm(Arm arm, double targetAngle) {
        this.arm = arm;
        this.targetAngle = targetAngle;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.initialize();
        arm.setSetpoint(targetAngle);
    }

    @Override
    public void execute() {
        arm.usePID();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopArm();
    }

    @Override
    public boolean isFinished() {
        return arm.atSetpoint() || arm.hasReachedUpperLimit();
    }
}
