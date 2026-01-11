package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;


/// Class that makes implementing things that should toggle when you press a button more easily
public class ButtonToggle {
    public final boolean isStickButton;
    public boolean toggle;
    public final Button button;
    private long lastPressed;

    private boolean wasHeld;


    public ButtonToggle(@NonNull Button button, boolean toggle) {
        this.toggle = toggle;
        this.button = button;
        this.isStickButton = button == Button.LEFT_STICK_BUTTON || button == Button.RIGHT_STICK_BUTTON;

        this.wasHeld = false;
        this.lastPressed = 0;
    }

    /// Updates the toggle based on the current controller input and returns the result.
    public boolean check(@NonNull GamepadEx ctrl) {
        boolean isHeld = ctrl.getButton(button);
        if(isStickButton) {
            if(!isHeld)
                return toggle;
            // stick buttons are really finicky and when held down they often will be read as
            // briefly not being held down so this adds tolerance so it doesn't constantly switch
            long now = System.currentTimeMillis();
            toggle ^= now - lastPressed > 100;
            lastPressed = now;
        } else {
            toggle ^= isHeld && !wasHeld;
            wasHeld = isHeld;
        }
        return toggle;
    }

    public boolean isToggledOn(){
        return toggle;
    }
}
