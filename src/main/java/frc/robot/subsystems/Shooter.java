package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.ShuffleboardControl;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public CANSparkMax bottomFlywheel;
  public RelativeEncoder bottomEncoder;
  public CANSparkMax topFlywheel;
  public RelativeEncoder topEncoder;
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
  double goalRPM = ShooterConstants.shooterRPM;

  public Shooter() {
    bottomFlywheel = new CANSparkMax(ShooterConstants.bottomFlywheelPort, MotorType.kBrushless);
    topFlywheel = new CANSparkMax(ShooterConstants.topFlywheelPort, MotorType.kBrushless);
    
    bottomEncoder = bottomFlywheel.getEncoder();
    topEncoder = topFlywheel.getEncoder();

    SmartDashboard.putNumber("Goal RPM", goalRPM);
    setMotorsToCoast();
  }

  void setMotorsToCoast(){
    bottomFlywheel.setIdleMode(IdleMode.kCoast);
    topFlywheel.setIdleMode(IdleMode.kCoast);
  }

  public double convertRPMToRPS(double rpm){
    return rpm / 60;
  }
  public void setGoalRPM(double rpm){ goalRPM = convertRPMToRPS(rpm); }
  public void shoot() { shoot(goalRPM); }

  public void shoot(double rpm) {
    double rps = convertRPMToRPS(rpm);
    double calculatedSpeed = feedforward.calculate(rps);

    bottomFlywheel.setVoltage(calculatedSpeed);
    topFlywheel.setVoltage(calculatedSpeed);
  }

  public void stopShooting() {
    bottomFlywheel.set(0);
    topFlywheel.set(0);
  }

  @Override
  public void periodic() {
    setGoalRPM( SmartDashboard.getNumber("Goal RPM", ShooterConstants.shooterRPM) );

    ShuffleboardControl.addToDevelopment("Low RPM", bottomEncoder.getVelocity());
    ShuffleboardControl.addToDevelopment("High RPM", topEncoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {}
}
