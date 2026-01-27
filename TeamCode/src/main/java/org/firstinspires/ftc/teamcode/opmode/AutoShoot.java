package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Shoot", group = "Main")
public class AutoShoot extends AutoSuperClass {
    @Override
    public void runOpMode() {
        telemetryPipeline.addHeader("Initialization");
        initialize(true);
//==========================//
        telemetryPipeline.addHeader("waiting for start");
        waitForStart();
        initializeCocker();
        sleep(100);
        telemetryPipeline.addHeader("Starting");
        move(-0.7, 0, 1000);
//==========================//
        sleep(5000);
        telemetryPipeline.addHeader("Threads");
        sleep(200);
        startShootingThread();
        sleep(5000);
        telemetryPipeline.addHeader("Shooting 1");
        shoot();
        sleep(5000);
        telemetryPipeline.addHeader("Shooting 2");
        shoot();
        sleep(5000);
        telemetryPipeline.addHeader("Shooting 2");
        shoot();
    }
}
