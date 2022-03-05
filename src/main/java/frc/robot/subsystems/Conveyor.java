package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.ConveyorConstants.ConveyorSection;

public class Conveyor extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  CANSparkMax entranceConveyor;
  CANSparkMax exitConveyor;

  DigitalInput entranceBeamBreak;
  DigitalInput exitBeamBreak;

  public Conveyor() {
    entranceConveyor = new CANSparkMax(ConveyorConstants.entranceConveyorPort, MotorType.kBrushless);
    exitConveyor = new CANSparkMax(ConveyorConstants.exitConveyorPort, MotorType.kBrushless);
    
    entranceConveyor.setIdleMode(IdleMode.kBrake);
    exitConveyor.setIdleMode(IdleMode.kBrake);

    entranceBeamBreak = new DigitalInput(ConveyorConstants.entranceBeamBreakPort);
    exitBeamBreak = new DigitalInput(ConveyorConstants.exitBeamBreakPort);
  }
  
  public void convey() {
    convey(ConveyorSection.ENTRANCE);
    convey(ConveyorSection.EXIT);
  }

  public void reverseConvey() {
    reverseConvey(ConveyorSection.ENTRANCE);
    reverseConvey(ConveyorSection.EXIT);
  }

  public void stopConveyor() {
    stopConveyor(ConveyorSection.ENTRANCE);
    stopConveyor(ConveyorSection.EXIT);
  }

  public void convey(ConveyorSection section){
    if(section == ConveyorSection.ENTRANCE) entranceConveyor.set(-ConveyorConstants.conveyorSpeed);
    else exitConveyor.set(ConveyorConstants.conveyorSpeed);
  }

  public void reverseConvey(ConveyorSection section) {
    if(section == ConveyorSection.ENTRANCE) entranceConveyor.set(ConveyorConstants.conveyorSpeed);
    else exitConveyor.set(-ConveyorConstants.conveyorSpeed);
  }

  public void stopConveyor(ConveyorSection section) {
    if(section == ConveyorSection.ENTRANCE) entranceConveyor.set(0);
    else exitConveyor.set(0);
  }

  public boolean getBeamBreakStatus(ConveyorSection section){
    return (section == ConveyorSection.ENTRANCE) ? !entranceBeamBreak.get() : exitBeamBreak.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Entrance", getBeamBreakStatus(ConveyorSection.ENTRANCE));
    SmartDashboard.putBoolean("Exit", getBeamBreakStatus(ConveyorSection.EXIT));
  }

  @Override
  public void simulationPeriodic() {}
}
