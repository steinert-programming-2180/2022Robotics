package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  AHRS navx;

  public GyroSubsystem() {
    navx = new AHRS(Port.kMXP);
  }

  public void resetAngle() {
    navx.zeroYaw();
  }

  // returns angle from -180 to 180
  public double getAngle() {
    return navx.getYaw();
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
