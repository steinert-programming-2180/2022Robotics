// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Required for some code to run
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
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

    setToBrake();
  }

  public void raise(ClimberSide side) {
    (side == ClimberSide.LEFT ? leftSpark : rightSpark).set(ClimberConstants.climbSpeed);
  }

  public void raise() {
    raise(ClimberSide.LEFT);
    raise(ClimberSide.RIGHT);
  }

  private CANSparkMax choosenSpark; // this is here so we can avoid a warning

  public void lower(ClimberSide side) {
    choosenSpark = (side == ClimberSide.LEFT) ? leftSpark : rightSpark;
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

  public void setToBrake() {
    leftSpark.setIdleMode(IdleMode.kBrake);
    rightSpark.setIdleMode(IdleMode.kBrake);
  }

  public void setToCoast() {
    leftSpark.setIdleMode(IdleMode.kCoast);
    rightSpark.setIdleMode(IdleMode.kCoast);
  }
}
