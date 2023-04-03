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
import frc.robot.ShuffleboardControl;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private CANSparkMax leftSpark, rightSpark;
  private DoubleSolenoid lockingSolenoid; // this solenoid will lock and/or unlock the climber

  public Climber() {
    leftSpark = new CANSparkMax(ClimberConstants.leftClimberPort, MotorType.kBrushless);
    rightSpark = new CANSparkMax(ClimberConstants.rightClimberPort, MotorType.kBrushless);

    lockingSolenoid = new DoubleSolenoid(Constants.PneumaticHubPort, PneumaticsModuleType.REVPH,
        ClimberConstants.extendSolenoid, ClimberConstants.retractSolenoid);

    leftSpark.follow(rightSpark, true);

    setSparksToBrake();
  }

  public void raise() {
    rightSpark.set(-ClimberConstants.climbSpeed);
  }

  public void lower() {
    rightSpark.set(ClimberConstants.climbSpeed);
  }

  public void lockPosition() {
    lockingSolenoid.set(Value.kForward);
  }

  public void unlockPosition() {
    lockingSolenoid.set(Value.kReverse);
  }

  public boolean isLocked() {
    return lockingSolenoid.get() == Value.kForward;
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

  public void setSparksToBrake() {
    leftSpark.setIdleMode(IdleMode.kBrake);
    rightSpark.setIdleMode(IdleMode.kBrake);
  }

  public void stop() {
    rightSpark.set(0);
  }

  @Override
  public void periodic() {
    stop();
  }
}
