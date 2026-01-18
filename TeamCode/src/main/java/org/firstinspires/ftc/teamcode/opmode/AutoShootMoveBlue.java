package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Shoot + Move (Blue)")
public class AutoShootMoveBlue extends AutoShoot {

    @Override
    public void runOpMode() {
        super.runOpMode();
        move(0.7,(2*Math.PI)/3, 1400);
    }
}
