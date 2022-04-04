package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class can be used when you need a trigger to act more like a button.
 * <p>
 * 
 * It inherits all button methods, so this can be used, for example, with the
 * triggers of an {@link XboxController} are pressed past a certain value.
 */

public class AxisTrigger extends Button {

    GenericHID joystick;
    int axis;
    double threshold;
    boolean greaterThan;

    /**
     * Creates an AxisTrigger object which triggers when the trigger is passed a
     * certain threshold value
     * 
     * @param joystick    The GenericHID object (usually a joystick)
     * @param axis        The integer value of the axis port
     * @param threshold   The threshold value
     * @param greaterThan If true, then the command will trigger when the axis is
     *                    greater than the threshold.
     *                    If false, then it will trigger when the axis is less than
     *                    the threshold
     */
    public AxisTrigger(GenericHID joystick, int axis, double threshold, boolean greaterThan) {
        this.joystick = joystick;
        this.axis = axis;
        this.threshold = threshold;
        this.greaterThan = greaterThan;
    }

    /**
     * Creates an AxisTrigger object which triggers when the trigger is passed a
     * certain threshold value,
     * which defaults to triggering above the threshold value
     * 
     * @param joystick  The GenericHID object (usually a joystick)
     * @param axis      The integer value of the axis port
     * @param threshold The threshold value
     */
    public AxisTrigger(GenericHID joystick, int axis, double threshold) {
        this(joystick, axis, threshold, true);
    }

    public boolean get() {
        double triggerVal = joystick.getRawAxis(axis);
        return greaterThan ? (triggerVal >= threshold) : (triggerVal <= threshold);
    }

}