package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Leak + Move")
public class AutoLeakMove extends AutoLeak {
    @Override
    public void runOpMode() {
        super.runOpMode();
        move(0.7,4*Math.PI/3, 600);
    }
}