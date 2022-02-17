// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Spark bottomFlywheel;
  public Spark topFlywheel;

  public Shooter() {
    bottomFlywheel = new Spark(ShooterConstants.bottomFlywheelPort);
    topFlywheel = new Spark(ShooterConstants.topFlywheelPort);
  }

  public void Shoot() {
    bottomFlywheel.set(ShooterConstants.shooterSpeed);
    topFlywheel.set(-ShooterConstants.shooterSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
