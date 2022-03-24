package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class RaiseArm extends CommandBase {
  Arm arm;
  double targetEncoderValue = ArmConstants.targetEncoderValue;
  double slowSpeedTakeOver = 15; // is in encoder value

  public RaiseArm(Arm arm) {
    this.arm = arm;
  }

  public RaiseArm(Arm arm, double targetEncoderValue){
    this.targetEncoderValue = targetEncoderValue;
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetEncoderValue = arm.getGoal();

    if(shouldBeSlow() && isArmUnderGoal()) {
      arm.moveArm(0.2);
    } else if(isArmUnderGoal()){
      arm.moveArm(0.4);
    } else {
      arm.moveArm(0);
    }
  }

  boolean isArmUnderGoal(){
    return arm.getEncoderValue() - targetEncoderValue < 0;
  }

  boolean isArmOverGoal(){
    return arm.getEncoderValue() - targetEncoderValue > 0;
  }

  boolean shouldBeSlow(){
    return Math.abs(arm.getEncoderValue() - targetEncoderValue) < slowSpeedTakeOver;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.moveArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.getEncoderValue() >= targetEncoderValue;
  }
}
