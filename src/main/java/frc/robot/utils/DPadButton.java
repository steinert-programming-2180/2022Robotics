package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class can be used when you need the D-Pad (POV) to run commands.
 * 
 * It inherits all button methods, so this can be used, for example, with the
 * D-Pad of an {@link XboxController}.
 */
public class DPadButton extends Button {

    GenericHID joystick;
    Direction direction;

    /**
     * Creates a DPadButton Object which triggers when a direction of the D-Pad is
     * pressed.
     * 
     * @param joystick  The GenericHID object (usually a joystick)
     * @param direction The {@link Direction} of the controller
     */
    public DPadButton(GenericHID joystick, Direction direction) {
        this.joystick = joystick;
        this.direction = direction;
    }

    public static enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270),
        UP_RIGHT(45), DOWN_RIGHT(135),
        DOWN_LEFT(225), UP_LEFT(315);

        int direction;

        private Direction(int direction) {
            this.direction = direction;
        }
    }

    public boolean get() {
        int dPadValue = joystick.getPOV();
        return (dPadValue == direction.direction);
    }

}