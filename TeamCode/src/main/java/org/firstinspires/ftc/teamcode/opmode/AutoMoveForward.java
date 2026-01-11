package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Move Forward")
public class AutoMoveForward extends AutoSuperClass {
    @Override
    public void runOpMode() {
        initialize(true);

//==========================//
        waitForStart();
//==========================//

        move(0.7,0, 600);
        stopFlyWheelNice();
    }
}
