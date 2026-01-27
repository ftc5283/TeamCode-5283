package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Move Forward", group = "Main")
public class AutoMoveForward extends AutoSuperClass {
    @Override
    public void runOpMode() {
        initialize(true);

//==========================//
        waitForStart();
//==========================//
//        initializeCocker();
//        Thread cockerThread = new Thread(() -> {
//            cockerMove.run();
//        });
////        cockerMove.targetPos
//        cockerThread.start();
        move(0.7,0, 600);

    }
}
