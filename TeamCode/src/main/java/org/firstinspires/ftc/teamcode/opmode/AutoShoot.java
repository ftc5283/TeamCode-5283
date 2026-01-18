package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Shoot", group = "Main")
public class AutoShoot extends AutoSuperClass {
    @Override
    public void runOpMode() {
        initialize(true);

//==========================//
        waitForStart();
//==========================//
        startThreads();
        sleep(500);
        shoot();
        shoot();
        shoot();
    }
}
