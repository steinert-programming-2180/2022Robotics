package frc.robot.utils;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.GenericHID;

public class AxisTrigger extends Button {

    GenericHID joystick;
    int axis;
    double threshold;
    boolean lessThan;

    public AxisTrigger(GenericHID joystick, int axis, double threshold, boolean lessThan) {
        this.joystick = joystick;
        this.axis = axis;
        this.threshold = threshold;
        this.lessThan = lessThan;
    }

    public AxisTrigger(GenericHID joystick, int axis, double threshold) {
        this(joystick, axis, threshold, false);
    }

    

    public boolean get() {
        double triggerVal = joystick.getRawAxis(axis);
        return !lessThan ? (triggerVal >= threshold) : (triggerVal <= threshold);
    }

}