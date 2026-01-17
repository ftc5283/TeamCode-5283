package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;


/// A class that makes it easier to have something happen only on the first frame someone pressed
/// a button rather than having it happen every frame it is being held down.
public class ButtonOnPress {
    public final boolean isStickButton;
    public boolean lastValue;
    public final Button button;
    public long lastPressed;

    private boolean wasHeld;


    public ButtonOnPress(@NonNull Button button) {
        this.button = button;
        this.isStickButton = button == Button.LEFT_STICK_BUTTON || button == Button.RIGHT_STICK_BUTTON;

        this.lastValue = false;
        this.wasHeld = false;
        this.lastPressed = 0;
    }

    /// Updates current state based on the controller input.
    /// Returns true when the button is first pressed.
    public boolean check(@NonNull GamepadEx ctrl) {
        boolean isHeld = ctrl.getButton(button);
        if(isStickButton) {
            long now = System.currentTimeMillis();
            boolean withinTolerance = now - lastPressed > 100;
            if(!isHeld || withinTolerance)
                return false;
            // stick buttons are really finicky and when held down they often will be read as
            // briefly not being held down so this adds tolerance so it doesn't constantly swap
            lastValue = true;
            lastPressed = now;
        } else {
            if (isHeld) {
                lastValue = !wasHeld;
                lastPressed = System.currentTimeMillis();
            }
            wasHeld = isHeld;
        }
        return lastValue;
    }

    public boolean checkWithin(@NonNull GamepadEx ctrl, long milliSeconds) {
        return check(ctrl) || (System.currentTimeMillis() <= lastPressed + milliSeconds);
    }

    /// Updates the toggle based on the controller input and returns the result.
    public boolean wasPressedLastCheck(){
        return lastValue;
    }
}