package org.firstinspires.ftc.teamcode.pipeline;

public final class WheelDirection {
    /** @noinspection unused*/
    public static final boolean FORWARD = true;
    /** @noinspection unused*/
    public static final boolean BACKWARD = false;


    ///Edit these whenever motors or bevel gears are changed.
    public static final boolean FL = BACKWARD;

    ///Edit these whenever motors or bevel gears are changed.
    public static final boolean FR = FORWARD;

    ///Edit these whenever motors or bevel gears are changed.
    public static final boolean BL = BACKWARD;

    ///Edit these whenever motors or bevel gears are changed.
    public static final boolean BR = FORWARD;

    public static final boolean INVERT_RIGHT_SIDE = !(FR && BR);

    public static final boolean INVERT_LEFT_SIDE = !(FL && BL);

    /** @noinspection unused*/
    public static String toString(boolean direction) {
        if (direction == FORWARD) {
            return "Forward";
        } else {
            return "Backward";
        }
    }

    static {
        //noinspection ConstantValue
        assert FL == BL : "Left side must have wheels going in the same direction.";
        //noinspection ConstantValue
        assert FR == BR : "Right side must have wheels going in the same direction.";
    }
}
