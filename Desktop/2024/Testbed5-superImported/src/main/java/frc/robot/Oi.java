package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Joystick;

public class Oi {
    
    private final PS4Controller drivController = new PS4Controller(0);

    public double GetDriverRawAxis(int axis) {
        return drivController.getRawAxis(axis);
    }

    public boolean XButtonPushed() {
        return drivController.getRawButton(1);
    }
    
    public boolean bButtonPushed() {
        return drivController.getRawButton(3);
    }

    public boolean bButtonReleased() {
        return drivController.getSquareButtonReleased();
    }


    public boolean aButtonPushed() {
        return drivController.getSquareButton();
    }

    // private final Joystick joyController = new Joystick(0);

    // public double GetJoyRawAxis(int axis) {
    //     return joyController.getRawAxis(axis);
    // }
}
