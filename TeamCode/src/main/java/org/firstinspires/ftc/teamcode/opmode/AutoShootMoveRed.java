package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Shoot + Move (Red)", group = "Main")
public class AutoShootMoveRed extends AutoShoot {

    @Override
    public void runOpMode() {
        super.runOpMode();
        move(0.7,-(2*Math.PI)/3, 1400, -0.05);
    }
}
