package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.actions.MotorActions;
import org.firstinspires.ftc.teamcode.pipeline.HardwarePipeline;
import org.firstinspires.ftc.teamcode.pipeline.TelemetryPipeline;
import org.firstinspires.ftc.teamcode.utility.HardwareConstants;
import org.firstinspires.ftc.teamcode.utility.Misc;
import org.firstinspires.ftc.teamcode.utility.Supervisor;
import org.firstinspires.ftc.teamcode.actions.MotorActions.MoveMotor;

public abstract class AutoSuperClass extends LinearOpMode {

    protected TelemetryPipeline telemetryPipeline;
    protected HardwarePipeline drive;

    DcMotorEx cocker, conveyor;
    ServoImplEx wall;
    MoveMotor cockerMove, conveyorMove;

    boolean shoot = false;
    boolean justFired = false;
    long conveyorTimer = 0;
    boolean startedLifting = false;
    boolean justLifted = false;
    long wallTimer = 0;

    public void shoot() {
        this.shoot = true;
        this.sleep(4100);
    }

    public void initialize(boolean startParallelTelemetry) {
        telemetryPipeline = new TelemetryPipeline(telemetry);
        drive = new HardwarePipeline(hardwareMap);
//
//        cocker = hardwareMap.get(DcMotorEx.class, "cocker");
//        cockerMove = new MotorActions(cocker, telemetryPipeline).moveMotor(
//            (cocker.getCurrentPosition()/HardwareConstants.COCKER_360)*HardwareConstants.COCKER_360 +
//            HardwareConstants.COCKER_360/6
//        );
//
//        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");
//        conveyorMove = new MotorActions(conveyor, telemetryPipeline).moveMotor(0);
//        conveyorMove.powerMultiplier = 0.25;

        if (startParallelTelemetry) {
            this.startParallelTelemetry();
        }

//        final AutoSuperClass op = this;
//        Thread loadAndLaunch = new Thread(() -> {
//            if (op.shoot) {
//                op.cockerMove.targetPos += HardwareConstants.COCKER_360;
//                op.justFired = true;
//                op.shoot = false;
//            }
//            op.cockerMove.run();
//
//            if ((op.justFired && op.cockerMove.within())) {
//                op.justFired = false;
//                op.conveyorTimer = System.currentTimeMillis();
//            }
//
//            if (op.conveyorTimer + 1000 >= System.currentTimeMillis()) {
//                op.conveyorMove.targetPos = HardwareConstants.CONVEYOR_TOP_POSITION;
//                op.telemetryPipeline.addDataPoint("Conveyor goal", op.conveyorMove.targetPos);
//                op.startedLifting = true;
//            } else {
//                if (op.startedLifting) {
//                    op.justLifted = true;
//                    op. startedLifting = false;
//                }
//                op.conveyorMove.targetPos = 70* Misc.sgn(HardwareConstants.CONVEYOR_TOP_POSITION);
//                op.telemetryPipeline.addDataPoint("Conveyor goal", conveyorMove.targetPos);
//            }
//            op.conveyorMove.run();
//
//            if (op.justLifted && op.conveyorMove.within()) {
//                op.justLifted = false;
//                op.wallTimer = System.currentTimeMillis();
//            }
//
//            if (wallTimer + 1000 >= System.currentTimeMillis()) {
//                op.wall.setPosition(HardwareConstants.WALL_POS);
//            } else {
//                op.wall.setPosition(0);
//            }
//        });
//        loadAndLaunch.start();
    }

    /// angle is the angle to the line extending out the front of the robot
    public void move(double speed, double angleRad, long timeMillis) {
        double forwardSpeed = Math.cos(angleRad) * speed;
        double strafeSpeed = Math.sin(angleRad) * speed;
        drive.driveRobotCentric(-strafeSpeed, -forwardSpeed, 0.1);
        sleep(timeMillis);
        drive.driveRobotCentric(0,0,0);
    }

    public void startParallelTelemetry() {
        AutoSuperClass auto = this;
        Thread telemetryThread = new Thread() {
            @Override
            public void run(){
                while (!auto.isStopRequested()) {
                    telemetryPipeline.addDataPointPerpetual("cocker pos", cocker.getCurrentPosition());
                    telemetryPipeline.addDataPointPerpetual("conveyor pos", conveyor.getCurrentPosition());
                    telemetryPipeline.addDataPointPerpetual("wall pos", wall.getPosition());
                    telemetryPipeline.refresh();
                }
            }
        };
        telemetryThread.start();
    }
}