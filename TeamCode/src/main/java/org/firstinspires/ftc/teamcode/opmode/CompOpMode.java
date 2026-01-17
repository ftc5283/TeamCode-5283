package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.actions.MotorActions;
import org.firstinspires.ftc.teamcode.pipeline.HardwarePipeline;
import org.firstinspires.ftc.teamcode.pipeline.TelemetryPipeline;
import org.firstinspires.ftc.teamcode.utility.ButtonOnPress;
import org.firstinspires.ftc.teamcode.utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.utility.GampadUtils;
import org.firstinspires.ftc.teamcode.utility.HardwareConstants;
import org.firstinspires.ftc.teamcode.utility.Misc;
import org.firstinspires.ftc.teamcode.utility.Supervisor;
import org.firstinspires.ftc.teamcode.actions.MotorActions.MoveMotor;

@TeleOp(name = "Competition TeleOp")
public class CompOpMode extends OpMode{
    protected TelemetryPipeline telemetryPipeline;
    protected HardwarePipeline drive;
    protected final Supervisor supervisor = new Supervisor();

    ElapsedTime timer;

    GamepadEx gamepadEx1, gamepadEx2;
    DcMotorEx cocker, conveyor;
    ServoImplEx wall;
    VoltageSensor controlHub;

    MoveMotor cockerMove, conveyorMove;
//    MotorActions cockerActions;

    @Override
    public void init() {
        telemetryPipeline = new TelemetryPipeline(telemetry);
        drive = new HardwarePipeline(hardwareMap);

        timer = new ElapsedTime();

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);


//        cocker = AutoSuperClass.getCocker(hardwareMap);
        cocker = hardwareMap.get(DcMotorEx.class, "cocker");
//        cocker.setPower(0.05);
//        cockerActions = new MotorActions(cocker, telemetryPipeline);
        cockerMove = new MotorActions(cocker, telemetryPipeline)
            .moveMotor(
                (cocker.getCurrentPosition()/HardwareConstants.COCKER_360)*HardwareConstants.COCKER_360 +
                HardwareConstants.COCKER_360/4
            );

        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");
        conveyorMove = new MotorActions(conveyor, telemetryPipeline).moveMotor(0);
        conveyorMove.powerMultiplier = 0.25;

        wall = hardwareMap.get(ServoImplEx.class, "wall");
        wall.setDirection(Servo.Direction.REVERSE);

        telemetryPipeline.addDataPointPerpetual("init position", cocker.getCurrentPosition());
        telemetryPipeline.addDataPointPerpetual("init target position", cocker.getTargetPosition());
        telemetryPipeline.addDataPointPerpetual("init tolerance", cocker.getTargetPositionTolerance());

        controlHub = hardwareMap.voltageSensor.get("Control Hub");

        primaryCtrl = gamepadEx1;
        secondaryCtrl = gamepadEx2;

        conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cocker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

//    @Override
//    public void start() {
//
//    }


    // PRIMARY CONTROLLER
    final ButtonToggle throttleLeftStickToggle = new ButtonToggle(LEFT_STICK_BUTTON, true);
    final ButtonOnPress wallXPress = new ButtonOnPress(X);
    final ButtonOnPress conveyorYPress = new ButtonOnPress(Y);
    final ButtonOnPress conveyorBPress = new ButtonOnPress(B);
    final ButtonOnPress cockerAPress = new ButtonOnPress(A);



    // SECONDARY CONTROLLER
    final ButtonOnPress stickPowerAPress = new ButtonOnPress(A);
    final ButtonOnPress precisePowerBPress = new ButtonOnPress(B);
    final ButtonOnPress posXPress = new ButtonOnPress(X);
    final ButtonToggle testRightStickToggle = new ButtonToggle(RIGHT_STICK_BUTTON, false);


    @SuppressWarnings("unused")
    final ButtonOnPress incrementOnPress = new ButtonOnPress(DPAD_UP);
    @SuppressWarnings("unused")
    final ButtonOnPress decrementOnPress = new ButtonOnPress(DPAD_DOWN);
    @SuppressWarnings("unused")
    final ButtonOnPress doubleDeltaOnPress = new ButtonOnPress(DPAD_RIGHT);
    @SuppressWarnings("unused")
    final ButtonOnPress halveDeltaOnPress = new ButtonOnPress(DPAD_LEFT);

    GamepadEx primaryCtrl, secondaryCtrl;

    @FunctionalInterface
    private interface Double2Double {double map(double x);}
    final boolean squareInputs = true;
    final Double2Double squareInputsCorrection = (squareInputs ? (double x) -> x : Math::sqrt);
    final double throttleStrafe = squareInputsCorrection.map(0.6);
    final double throttleForwardBack = squareInputsCorrection.map(0.6);
    final double throttleTurn = squareInputsCorrection.map(0.55);

    boolean isCocked = false;
    boolean justFired = false;
    long conveyorTimer = 0;
    boolean startedLifting = false;
    boolean justLifted = false;
    long wallTimer = 0;

    @Override
    public void loop() {
        if (gamepad1.back) {
            primaryCtrl = gamepadEx1;
            secondaryCtrl = gamepadEx2;
        } else if (gamepad2.back) {
            primaryCtrl = gamepadEx2;
            secondaryCtrl = gamepadEx1;
        }

        // Don't get rid of this. Very useful for figuring out if the issue is the configuration or
        // if one of the bevel gears is on backwards.
        if (testRightStickToggle.check(secondaryCtrl)) {
            boolean fl = secondaryCtrl.getButton(A),
                    fr = secondaryCtrl.getButton(B),
                    bl = secondaryCtrl.getButton(X),
                    br = secondaryCtrl.getButton(Y);
            drive.driveWithMotorPowers(fl ? 1 : 0, fr ? 1 : 0, bl ? 1 : 0, br ? 1 : 0);

            telemetryPipeline.addHeader("PRESS RIGHT STICK ON SECONDARY CONTROLLER TO EXIT");
            telemetryPipeline.addDataPoint("Testing", "Testing");
            telemetryPipeline.addDataPoint("fl", fl);
            telemetryPipeline.addDataPoint("fr", fr);
            telemetryPipeline.addDataPoint("bl", bl);
            telemetryPipeline.addDataPoint("br", br);

            telemetryPipeline.refresh();
            return;
        }

        final double turnSpeed = gamepad1.right_trigger - gamepad1.left_trigger;
        final double[] speeds = GampadUtils.speedInputs(primaryCtrl);
        final double forwardSpeed = speeds[0];
        final double strafeSpeed = speeds[1];
        final boolean throttleSpeed = throttleLeftStickToggle.check(primaryCtrl);

        drive.driveRobotCentric(
            -strafeSpeed * (throttleSpeed ? throttleStrafe : 1),
            -forwardSpeed * (throttleSpeed ? throttleForwardBack : 1),
            -turnSpeed * (throttleSpeed ? throttleTurn : 1),
            squareInputs
        );

        if (cockerAPress.check(primaryCtrl) || this.justFired) {
//            if (isCocked) {
//                cockerMove.targetPos += 3*HardwareConstants.COCKER_360/4;
//                this.justFired = true;
//            } else {
//                cockerMove.targetPos += HardwareConstants.COCKER_360/4;
//            }
//            isCocked = !isCocked;
            cockerMove.targetPos += HardwareConstants.COCKER_360;
            this.justFired = true;
        }
        cockerMove.run();

        if (conveyorYPress.check(primaryCtrl) ||  (justFired && cockerMove.within())) {
            this.justFired = false;
            conveyorTimer = System.currentTimeMillis();
        }

        if (conveyorTimer + 1000 >= System.currentTimeMillis()) {
            conveyorMove.targetPos = HardwareConstants.CONVEYOR_TOP_POSITION;
            telemetryPipeline.addDataPoint("Conveyor goal", conveyorMove.targetPos);
            startedLifting = true;
        } else {
            if (startedLifting) {
                justLifted = true;
                startedLifting = false;
            }
            conveyorMove.targetPos = 70*Misc.sgn(HardwareConstants.CONVEYOR_TOP_POSITION);
            telemetryPipeline.addDataPoint("Conveyor goal", conveyorMove.targetPos);
        }
        conveyorMove.run();

        if (wallXPress.check(primaryCtrl) || (justLifted && conveyorMove.within())) {
            justLifted = false;
            wallTimer = System.currentTimeMillis();
        }

        if (wallTimer + 1000 >= System.currentTimeMillis()) {
            wall.setPosition(HardwareConstants.WALL_POS);
        } else {
            wall.setPosition(0);
        }

        supervisor.run(telemetryPipeline);

        telemetryPipeline.addDataPoint("forward Speed", forwardSpeed);
        telemetryPipeline.addDataPoint("turn Speed", turnSpeed);
        telemetryPipeline.addDataPoint("strafe Speed", strafeSpeed);

        telemetryPipeline.addDataPoint("y held", primaryCtrl.getButton(Y));
        telemetryPipeline.addDataPoint("b held", primaryCtrl.getButton(B));

        telemetryPipeline.addDataPoint("wall pos", wall.getPosition());
        telemetryPipeline.addDataPoint("wall pos", wall.getDirection());

        telemetryPipeline.addDataPoint("conveyor pos", conveyor.getCurrentPosition());
        telemetryPipeline.addDataPoint("conveyor target", conveyorMove.getTarget());
        telemetryPipeline.addDataPoint("cocker pos", cocker.getCurrentPosition());
        telemetryPipeline.addDataPoint("cocker target", cockerMove.getTarget());

        telemetryPipeline.refresh();
    }
}