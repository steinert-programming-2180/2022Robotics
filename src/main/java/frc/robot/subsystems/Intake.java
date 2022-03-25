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
    private CANSparkMax leftSpark, rightSpark;
    private DoubleSolenoid solenoid;

    public Intake(){
        leftSpark = new CANSparkMax(IntakeConstants.leftIntakePort, MotorType.kBrushless);
        rightSpark = new CANSparkMax(IntakeConstants.rightIntakePort, MotorType.kBrushless);
        leftSpark.follow(rightSpark, true);

        solenoid = new DoubleSolenoid(Constants.PneumaticHubPort, PneumaticsModuleType.REVPH, 1, 3);
        solenoid.set(Value.kForward);
    }

    public void intakeStop(){
        rightSpark.set(0);
    }

    public void intakeSpin(){ 
        rightSpark.set(-1);
    }

    public void intakeReverse(){      
        rightSpark.set(1);
   }

    public void extendOrRetract(){
        switch (solenoid.get()) {
            case kForward:
                retractIntake();
                break;
            case kReverse:
                extendIntake();
                break;
            default:
                extendIntake();
                break;
        }
    }

    public void extendIntake(){
        solenoid.set(Value.kForward);
    }

    public void retractIntake(){
        solenoid.set(Value.kReverse);
    }
}
