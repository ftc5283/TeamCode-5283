package org.firstinspires.ftc.teamcode.utility;

public final class Misc {
    public static double cast(boolean b) {
        return b ? 1 : 0;
    }
    public static boolean withinTolerance(int a, int b, int tolerance) {
        return  b - tolerance < a && a < b + tolerance;
    }
}
