package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Leak")
public class AutoLeak extends AutoSuperClass {
    @Override
    public void runOpMode() {
        initialize(true);

//==========================//
        waitForStart();
//==========================//

        leak();
    }
}