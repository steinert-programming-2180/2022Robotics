// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Required for some code to run
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberSide;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private CANSparkMax leftSpark, rightSpark;
  private DoubleSolenoid lockingSolenoid; // this solenoid will lock and/or unlock the climber

  public Climber() {
    leftSpark = new CANSparkMax(ClimberConstants.leftClimberPort, MotorType.kBrushless);
    rightSpark = new CANSparkMax(ClimberConstants.rightClimberPort, MotorType.kBrushless);

    lockingSolenoid = new DoubleSolenoid(Constants.PneumaticHubPort, PneumaticsModuleType.REVPH,
        ClimberConstants.extendSolenoid, ClimberConstants.retractSolenoid);

    leftSpark.setInverted(false);
    rightSpark.setInverted(false);
  }

  public void raise(ClimberSide side) {
    (side == ClimberSide.LEFT ? leftSpark : rightSpark).set(ClimberConstants.climbSpeed);
  }

  public void raise() {
    raise(ClimberSide.LEFT);
    raise(ClimberSide.RIGHT);
  }

  public void lower(ClimberSide side) {
    CANSparkMax choosenSpark = (side == ClimberSide.LEFT) ? leftSpark : rightSpark;
    choosenSpark.set(-ClimberConstants.climbSpeed);
  }

  public void lower() {
    lower(ClimberSide.LEFT);
    lower(ClimberSide.RIGHT);
  }

  public void lockPosition() {
    lockingSolenoid.set(Value.kForward);
  }

  public void unlockPosition() {
    lockingSolenoid.set(Value.kReverse);
  }

  public void toggleLock() {
    switch (lockingSolenoid.get()) {
      case kForward:
        lockingSolenoid.set(Value.kReverse);
        break;
      case kReverse:
      default:
        lockingSolenoid.set(Value.kForward);
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
