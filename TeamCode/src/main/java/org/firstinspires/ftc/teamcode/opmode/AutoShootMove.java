package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Shoot1 + Move")
public class AutoShootMove extends AutoShoot {

    @Override
    public void runOpMode() {
        super.runOpMode();
        move(0.7,(2*Math.PI)/3, 1000);
    }
}
