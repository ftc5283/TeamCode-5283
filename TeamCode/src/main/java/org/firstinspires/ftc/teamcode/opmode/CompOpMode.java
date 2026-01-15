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
import org.firstinspires.ftc.teamcode.utility.HardwareConstants;
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
    DcMotorEx cocker;
    VoltageSensor controlHub;

    @Override
    public void init() {
        telemetryPipeline = new TelemetryPipeline(telemetry);
        drive = new HardwarePipeline(hardwareMap);

        timer = new ElapsedTime();

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        cocker = hardwareMap.get(DcMotorEx.class, "cocker");
        controlHub = hardwareMap.voltageSensor.get("Control Hub");

        primaryCtrl = gamepadEx1;
        secondaryCtrl = gamepadEx2;

        cocker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cocker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // PRIMARY CONTROLLER
    final ButtonToggle intakeXToggle = new ButtonToggle(X, false);
    final ButtonToggle flyWheelRBumperToggle = new ButtonToggle(RIGHT_BUMPER, false);
    final ButtonToggle kickerBToggle = new ButtonToggle(B, false);
    final ButtonToggle testRightToggle = new ButtonToggle(RIGHT_STICK_BUTTON, false);
    final ButtonToggle throttleLeftStickToggle = new ButtonToggle(LEFT_STICK_BUTTON, true);

    // SECONDARY CONTROLLER
    final ButtonToggle holdFloorYToggle = new ButtonToggle(Y, false);


    @SuppressWarnings("unused")
    final ButtonOnPress incrementOnPress = new ButtonOnPress(DPAD_UP);
    @SuppressWarnings("unused")
    final ButtonOnPress decrementOnPress = new ButtonOnPress(DPAD_DOWN);
    @SuppressWarnings("unused")
    final ButtonOnPress doubleDeltaOnPress = new ButtonOnPress(DPAD_RIGHT);
    @SuppressWarnings("unused")
    final ButtonOnPress halveDeltaOnPress = new ButtonOnPress(DPAD_LEFT);

    GamepadEx primaryCtrl, secondaryCtrl;
    double flyWheelCurrent = 0;

    @FunctionalInterface
    private interface Double2Double {double map(double x);}
    final boolean squareInputs = false;
    final Double2Double squareInputsCorrection = (squareInputs ? (double x) -> x : Math::sqrt);
    final double throttleStrafe = squareInputsCorrection.map(0.5);
    final double throttleForwardBack = squareInputsCorrection.map(0.5);
    final double throttleTurn = squareInputsCorrection.map(0.5);

    double cockerPosDelta = 1;

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
        if (testRightToggle.check(primaryCtrl)) {
            boolean fl = primaryCtrl.getButton(A),
                    fr = primaryCtrl.getButton(B),
                    bl = primaryCtrl.getButton(X),
                    br = primaryCtrl.getButton(Y);
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

        if (doubleDeltaOnPress.check(secondaryCtrl)) {
            cockerPosDelta *= 2;
        } else if (halveDeltaOnPress.check(secondaryCtrl)) {
            cockerPosDelta /= 2;
        }
        if (incrementOnPress.check(primaryCtrl)) {
            HardwareConstants.COCKER_POS_A += cockerPosDelta;
        } else if (decrementOnPress.check(primaryCtrl)) {
            HardwareConstants.COCKER_POS_A -= cockerPosDelta;
        }

        final double turnSpeed = gamepad1.right_trigger - gamepad1.left_trigger;
        final double strafeSpeed = gamepadEx1.getLeftX();
        final double forwardSpeed = gamepadEx1.getLeftY();
        final boolean throttleSpeed = throttleLeftStickToggle.check(primaryCtrl);

        drive.driveRobotCentric(
                -strafeSpeed * (throttleSpeed ? throttleStrafe : 1),
                -forwardSpeed * (throttleSpeed ? throttleForwardBack : 1),
                -turnSpeed * (throttleSpeed ? throttleTurn : 1),
                true
        );

        supervisor.run(telemetryPipeline);
            
        telemetryPipeline.addDataPoint("forward Speed", forwardSpeed);
        telemetryPipeline.addDataPoint("turn Speed", turnSpeed);
        telemetryPipeline.addDataPoint("strafe Speed", strafeSpeed);
        telemetryPipeline.addDataPoint("current cocker pos", cocker.getCurrentPosition());
        telemetryPipeline.addDataPoint("real cocker target", cocker.getTargetPosition());
        telemetryPipeline.addDataPoint("set cocker target", HardwareConstants.COCKER_POS_A);
        telemetryPipeline.refresh();
    }
}