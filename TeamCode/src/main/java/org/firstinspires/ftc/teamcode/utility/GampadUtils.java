package org.firstinspires.ftc.teamcode.utility;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;
import static org.firstinspires.ftc.teamcode.utility.Misc.cast;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pipeline.TelemetryPipeline;
import static java.lang.Math.abs;
public class GampadUtils {
    /**
     *
     * @return true if any button on the dpad is held
     */
    public static boolean dpadInUse(@NonNull GamepadEx ctrl) {
        return ctrl.getButton(DPAD_UP) || ctrl.getButton(DPAD_DOWN) || ctrl.getButton(DPAD_LEFT) || ctrl.getButton(DPAD_RIGHT);
    }


    /**
     *  Checks if the left stick is being used.
     *
     * @param driftTolerance how much the position must differ in either direction before it is considered to be being moved by hand and not drift.
     *
     * @return true if the left stick is in use.
     */
    public static boolean lStickInUse(@NonNull GamepadEx ctrl, double driftTolerance) {
        return abs(ctrl.getLeftX()) <= driftTolerance || abs(ctrl.getLeftY()) <= driftTolerance || ctrl.isDown(LEFT_STICK_BUTTON);
    }

    /**
     *  Checks if the right stick is being used.
     *
     * @param driftTolerance how much the position must differ in either direction before it is considered to be being moved by hand and not drift.
     *
     * @return true if the right stick is in use.
     */
    public static boolean rStickInUse(@NonNull GamepadEx ctrl, double driftTolerance) {
        return abs(ctrl.getRightX()) <= driftTolerance || abs(ctrl.getRightY()) <= driftTolerance || ctrl.getButton(RIGHT_STICK_BUTTON);
    }

    /**
     * Computes speed inputs based on the dpad and left joystick of the controller.
     *
     * @return the speeds that it should move as [forwardSpeed, strafeSpeed].
     */
    public static double[] speedInputs(@NonNull GamepadEx ctrl) {
        double[] speeds = new double[2];
        if (dpadInUse(ctrl)) {
            speeds[0] = cast(ctrl.getButton(DPAD_UP)) - cast(ctrl.getButton(DPAD_DOWN));
            speeds[1] = cast(ctrl.getButton(DPAD_RIGHT)) - cast(ctrl.getButton(DPAD_LEFT));
        } else {
            speeds[0] = ctrl.getLeftY();
            speeds[1] = ctrl.getLeftX();
        }
        return speeds;
    }

    /**
     * Computes speed inputs based on the dpad and left joystick of the controller.
     * Adds telemetry for whether the dpad input or joystick is being checked.
     *
     * @return the speeds that it should move as [forwardSpeed, strafeSpeed].
     */
    public static double[] speedInputs(@NonNull GamepadEx ctrl, @NonNull TelemetryPipeline telemetryPipeline) {
        double[] speeds = new double[2];
        if (dpadInUse(ctrl)) {
            speeds[0] = cast(ctrl.getButton(DPAD_UP)) - cast(ctrl.getButton(DPAD_DOWN));
            speeds[1] = cast(ctrl.getButton(DPAD_RIGHT)) - cast(ctrl.getButton(DPAD_LEFT));
            telemetryPipeline.addDataPoint("Speed Ctrl", "D-Pad");
        } else {
            speeds[0] = -ctrl.getLeftY();
            speeds[1] = ctrl.getLeftX();
            telemetryPipeline.addDataPoint("Speed Ctrl", "Joystick");
        }
        return speeds;
    }

    public static String toString(GamepadKeys.Button button) {
        switch (button) {
            case A:
                return "A";
            case B:
                return "B";
            case X:
                return "X";
            case Y:
                return "Y";
            case LEFT_BUMPER:
                return "LEFT_BUMPER";
            case RIGHT_BUMPER:
                return "RIGHT_BUMPER";
            case DPAD_UP:
                return "DPAD_UP";
            case DPAD_DOWN:
                return "DPAD_DOWN";
            case DPAD_LEFT:
                return "DPAD_LEFT";
            case DPAD_RIGHT:
                return "DPAD_RIGHT";
            case BACK:
                return "BACK";
            case START:
                return "START";
            case LEFT_STICK_BUTTON:
                return "LEFT_STICK_BUTTON";
            case RIGHT_STICK_BUTTON:
                return "RIGHT_STICK_BUTTON";
            default:
                return "ERROR: UNKNOWN BUTTON";
        }
    }
}
