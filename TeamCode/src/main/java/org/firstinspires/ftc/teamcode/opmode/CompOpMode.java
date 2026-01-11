package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pipeline.HardwarePipeline;
import org.firstinspires.ftc.teamcode.pipeline.TelemetryPipeline;
import org.firstinspires.ftc.teamcode.utility.ButtonOnPress;
import org.firstinspires.ftc.teamcode.utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.utility.Supervisor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import static java.lang.Math.abs;
import static java.lang.Math.signum;

@TeleOp(name = "Competition TeleOp")
public class CompOpMode extends OpMode{
    protected TelemetryPipeline telemetryPipeline;
    protected HardwarePipeline drive;
    protected final Supervisor supervisor = new Supervisor();

    ElapsedTime timer;

    GamepadEx gamepadEx1, gamepadEx2;
    CRServoImplEx fwl, fwr, intake;
    ServoImplEx kicker, floor;
    DcMotorEx flyWheel;
    VoltageSensor controlHub;

    @Override
    public void init() {
        telemetryPipeline = new TelemetryPipeline(telemetry);
        drive = new HardwarePipeline(hardwareMap);

        timer = new ElapsedTime();

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        intake = hardwareMap.get(CRServoImplEx.class, "intake");
        fwr = hardwareMap.get(CRServoImplEx.class, "fwr");
        fwl = hardwareMap.get(CRServoImplEx.class, "fwl");
        kicker = hardwareMap.get(ServoImplEx.class, "kicker");
        floor = hardwareMap.get(ServoImplEx.class, "floor");
//        kicker = hardwareMap.get(ServoImplEx.class, "kicker");

        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        controlHub = hardwareMap.voltageSensor.get("Control Hub");

        ctrl = gamepadEx1;

        /*
           this might be the one time this is a bad idea, this might stress the motor a lot lol.
         */
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    final ButtonToggle intakeXToggle = new ButtonToggle(X, false);
    final ButtonToggle flyWheelRBumperToggle = new ButtonToggle(RIGHT_BUMPER, false);
    final ButtonToggle kickerBToggle = new ButtonToggle(B, false);
    final ButtonToggle testRightToggle = new ButtonToggle(RIGHT_STICK_BUTTON, false);
    final ButtonToggle throttleLeftStickToggle = new ButtonToggle(LEFT_STICK_BUTTON, true);
    final ButtonOnPress floorYOnPress = new ButtonOnPress(Y);
    GamepadEx ctrl;
    double flyWheelCurrent = 0;

    @FunctionalInterface
    private interface Double2Double {double map(double x);}
    final boolean squareInputs = false;
    final Double2Double squareInputsCorrection = (squareInputs ? (double x) -> x : Math::sqrt);
    final double throttleStrafe = squareInputsCorrection.map(0.5);
    final double throttleForwardBack = squareInputsCorrection.map(0.5);
    final double throttleTurn = squareInputsCorrection.map(0.5);


    @Override
    public void loop() {
        if (gamepad1.back) {
            ctrl = gamepadEx1;
        } else if (gamepad2.back) {
            ctrl = gamepadEx2;
        }
        // Don't get rid of this. Very useful for figuring out if the issue is the configuration or
        // if one of the bevel gears is on backwards.
        if (testRightToggle.check(ctrl)) {
            boolean fl = ctrl.getButton(A),
                    fr = ctrl.getButton(B),
                    bl = ctrl.getButton(X),
                    br = ctrl.getButton(Y);
            drive.driveWithMotorPowers(fl ? 1 : 0, fr ? 1 : 0, bl ? 1 : 0, br ? 1 : 0);

            telemetryPipeline.addHeader("PRESS RIGHT STICK TO EXIT");
            telemetryPipeline.addDataPoint("Testing", "Testing");
            telemetryPipeline.addDataPoint("fl", fl);
            telemetryPipeline.addDataPoint("fr", fr);
            telemetryPipeline.addDataPoint("bl", bl);
            telemetryPipeline.addDataPoint("br", br);

            telemetryPipeline.refresh();
            return;
        }


//        if (up.check(ctrl)) {
//            floorPos += 0.005;
//        } else if (down.check(ctrl)) {
//            floorPos -= 0.005;
//        }

        fwr.setPower(ctrl.getButton(LEFT_BUMPER) ? -1 : 0);
        fwl.setPower(ctrl.getButton(LEFT_BUMPER) ? 1 : 0);

        intake.setPower(intakeXToggle.check(ctrl) ? 1 : 0);


        kicker.setPosition(kickerBToggle.check(ctrl)? 0.60 : 0);
//        floor.setPosition(floorPos);

        if (
            floorYOnPress.check(ctrl) ||
            System.currentTimeMillis() < floorYOnPress.lastPressed + 1000
        ) {
            floor.setPosition(0.024);
        } else {
            floor.setPosition(0);
        }



        double turnSpeed = gamepad1.right_trigger - gamepad1.left_trigger;
        double strafeSpeed = gamepadEx1.getLeftX();
        double forwardSpeed = gamepadEx1.getLeftY();
        boolean throttleSpeed = throttleLeftStickToggle.check(ctrl);


        drive.driveRobotCentric(
                -strafeSpeed * (throttleSpeed ? throttleStrafe : 1),
                -forwardSpeed * (throttleSpeed ? throttleForwardBack : 1),
                -turnSpeed * (throttleSpeed ? throttleTurn : 1),
                true
        );

        supervisor.run(telemetryPipeline);

        flyWheel.setVelocity(flyWheelRBumperToggle.check(ctrl) ? 2.75 : 0, AngleUnit.RADIANS);

        flyWheelCurrent = flyWheel.getCurrent(CurrentUnit.MILLIAMPS);

        telemetryPipeline.addDataPoint("forward Speed", forwardSpeed);
        telemetryPipeline.addDataPoint("turn Speed", turnSpeed);
        telemetryPipeline.addDataPoint("strafe Speed", strafeSpeed);
        telemetryPipeline.addDataPoint("FlyWheel Current (mA)", flyWheelCurrent);
        telemetryPipeline.addDataPoint("FlyWheel Power", flyWheel.getPower());
        telemetryPipeline.addDataPoint("FlyWheel Velocity", flyWheel.getVelocity(AngleUnit.RADIANS));
        telemetryPipeline.refresh();
    }
}