package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    
    private TalonSRX leftInt, rightInt;
    public Intake(){
        leftInt= new TalonSRX(0);
        rightInt= new TalonSRX(1);        
    }

    public void intakeStop(){
        
        leftInt.set(ControlMode.PercentOutput, 0);
        rightInt.set(ControlMode.PercentOutput, 0);
    }

    public void intakeSpin(){ 
        leftInt.set(ControlMode.PercentOutput, 1);
        rightInt.set(ControlMode.PercentOutput, -1);
    }

    public void intakeReverse(){      
        leftInt.set(ControlMode.PercentOutput, -1);
        rightInt.set(ControlMode.PercentOutput, 1);
    }

}
