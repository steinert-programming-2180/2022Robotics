// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Spark bottomS;
  public Spark topS;

  public Shooter() {
    bottomS = new Spark(0);
    topS = new Spark(1);
  }

  public void shoot() {
    bottomS.set(1);
    topS.set(-1);
  }

  public void stopshoot() {
    bottomS.set(0);
    topS.set(0);
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
