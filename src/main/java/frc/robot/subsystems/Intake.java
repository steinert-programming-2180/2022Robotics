package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private CANSparkMax leftInt, rightInt;
    private DoubleSolenoid solenoid;

    public Intake(){
        leftInt = new CANSparkMax(IntakeConstants.leftIntakePort, MotorType.kBrushless);
        rightInt = new CANSparkMax(IntakeConstants.rightIntakePort, MotorType.kBrushless);
        solenoid = new DoubleSolenoid(Constants.PneumaticHubPort, PneumaticsModuleType.REVPH, 1, 3);        
    }

    public void intakeStop(){
        leftInt.set(0);
        rightInt.set(0);
    }

    public void intakeSpin(){ 
        leftInt.set(-1);
        rightInt.set(-1);
    }

    public void intakeReverse(){      
        leftInt.set(1);
        rightInt.set(1);
    }

    public void extendIntake(){
        solenoid.set(Value.kForward);
    }

    public void retracktIntake(){
        solenoid.set(Value.kReverse);
    }
}
