// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Spark entranceConveyor;
  public Spark exitConveyor;

  public Conveyor() {
    entranceConveyor = new Spark(ConveyorConstants.entranceConveyorPort);
    exitConveyor = new Spark(ConveyorConstants.exitConveyorPort);
  }

  public void Convey() {
    entranceConveyor.set(ConveyorConstants.conveyorSpeed);
    exitConveyor.set(-ConveyorConstants.conveyorSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during
  }
}
