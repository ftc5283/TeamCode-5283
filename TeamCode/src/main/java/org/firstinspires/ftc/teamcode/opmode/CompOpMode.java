package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pipeline.HardwarePipeline;
import org.firstinspires.ftc.teamcode.pipeline.TelemetryPipeline;
import org.firstinspires.ftc.teamcode.utility.ButtonOnPress;
import org.firstinspires.ftc.teamcode.utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.utility.GampadUtils;
import org.firstinspires.ftc.teamcode.utility.HardwareConstants;
import org.firstinspires.ftc.teamcode.utility.Supervisor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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

    @Override
    public void init() {
        telemetryPipeline = new TelemetryPipeline(telemetry);
        drive = new HardwarePipeline(hardwareMap);

        timer = new ElapsedTime();

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);


//        cocker = AutoSuperClass.getCocker(hardwareMap);
        cocker = hardwareMap.get(DcMotorEx.class, "cocker");
        cocker.setPower(0.05);
        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");
        wall = hardwareMap.get(ServoImplEx.class, "wall");

        telemetryPipeline.addDataPointPerpetual("init position", cocker.getCurrentPosition());
        telemetryPipeline.addDataPointPerpetual("init target position", cocker.getTargetPosition());
        telemetryPipeline.addDataPointPerpetual("init tolerance", cocker.getTargetPositionTolerance());

        controlHub = hardwareMap.voltageSensor.get("Control Hub");

        primaryCtrl = gamepadEx1;
        secondaryCtrl = gamepadEx2;

        try {
            Thread.sleep(1000);
        } catch (InterruptedException ignored){}
    }

    @Override
    public void start() {
        cocker.setPower(0);
        cocker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyor.setTargetPosition(HardwareConstants.CONVEYOR_POS);
        conveyor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // PRIMARY CONTROLLER
    final ButtonToggle throttleLeftStickToggle = new ButtonToggle(LEFT_STICK_BUTTON, true);
    final ButtonOnPress wallXPress = new ButtonOnPress(X);


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
    final double throttleStrafe = squareInputsCorrection.map(0.5);
    final double throttleForwardBack = squareInputsCorrection.map(0.5);
    final double throttleTurn = squareInputsCorrection.map(0.5);

    int conveyorPosDelta = 1;
//    double conveyorPowerDelta = 0.25;
//
//    int mode = -1;

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

        if (wallXPress.checkWithin(primaryCtrl, 2500)) {
            telemetryPipeline.addDataPoint("wall", "should be 90 deg");
            wall.setPosition(90.0/300);
        } else {
            telemetryPipeline.addDataPoint("wall", "should be 0 deg");
            wall.setPosition(0);
        }
        telemetryPipeline.addDataPoint("wall pos", wall.getPosition());
        telemetryPipeline.addDataPoint("wall connect info", wall.getConnectionInfo());
        telemetryPipeline.addDataPoint("wall pos", wall.getDirection());
        telemetryPipeline.addDataPoint("wall pwm", wall.isPwmEnabled());
        telemetryPipeline.addDataPoint("wall pwm", wall.getPwmRange());
        telemetryPipeline.addDataPoint("wall ctrl", wall.getController());

        final double turnSpeed = gamepad1.right_trigger - gamepad1.left_trigger;
//        final double forwardSpeed = gamepadEx1.getLeftY();
//        final double strafeSpeed = gamepadEx1.getLeftX();
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

        supervisor.run(telemetryPipeline);

//        telemetryPipeline.addDataPoint("forward Speed", forwardSpeed);
//        telemetryPipeline.addDataPoint("turn Speed", turnSpeed);
//        telemetryPipeline.addDataPoint("strafe Speed", strafeSpeed);

        telemetryPipeline.addDataPoint("current cocker pos", cocker.getCurrentPosition());
        telemetryPipeline.addDataPoint("cocker current (mA)", cocker.getCurrent(CurrentUnit.MILLIAMPS));
        telemetryPipeline.addDataPoint("max safe current (mA)", cocker.getCurrentAlert(CurrentUnit.MILLIAMPS));
        telemetryPipeline.addDataPoint("real cocker power", cocker.getPower());



        if (doubleDeltaOnPress.check(secondaryCtrl)) {
            conveyorPosDelta *= 2;
        } else if (halveDeltaOnPress.check(secondaryCtrl)) {
            conveyorPosDelta /= 2;
        }
        if (incrementOnPress.check(secondaryCtrl)) {
            HardwareConstants.CONVEYOR_POS += conveyorPosDelta;
        } else if (decrementOnPress.check(secondaryCtrl)) {
            HardwareConstants.CONVEYOR_POS -= conveyorPosDelta;
        }

        conveyor.setTargetPosition(HardwareConstants.CONVEYOR_POS);

//        telemetryPipeline.addDataPoint("MODE", "target");
        telemetryPipeline.addDataPoint("current conveyor pos", cocker.getCurrentPosition());
        telemetryPipeline.addDataPoint("real conveyor target", conveyor.getTargetPosition());
        telemetryPipeline.addDataPoint("intended conveyor target", HardwareConstants.CONVEYOR_POS);
        telemetryPipeline.addDataPoint("conveyor target delta", conveyorPosDelta);
        telemetryPipeline.addDataPoint("tolerance", conveyor.getTargetPositionTolerance());

//        if (posXPress.check(secondaryCtrl)) {
//            mode = 0;
//        } else if (precisePowerBPress.check(secondaryCtrl)) {
//            mode = 1;
//        } else if (stickPowerAPress.check(secondaryCtrl)) {
//            mode = 2;
//        }
//        switch (mode) {
//            case 0:
//                if (doubleDeltaOnPress.check(secondaryCtrl)) {
//                    cockerPosDelta *= 2;
//                } else if (halveDeltaOnPress.check(secondaryCtrl)) {
//                    cockerPosDelta /= 2;
//                }
//                if (incrementOnPress.check(secondaryCtrl)) {
//                    HardwareConstants.CONVEYOR_POS += cockerPosDelta;
//                } else if (decrementOnPress.check(secondaryCtrl)) {
//                    HardwareConstants.CONVEYOR_POS -= cockerPosDelta;
//                }
//
//                cocker.setTargetPosition(HardwareConstants.CONVEYOR_POS);
//                cocker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                telemetryPipeline.addDataPoint("MODE", "target");
//                telemetryPipeline.addDataPoint("real cocker target", cocker.getTargetPosition());
//                telemetryPipeline.addDataPoint("intended cocker target", HardwareConstants.CONVEYOR_POS);
//                telemetryPipeline.addDataPoint("cocker target delta", cockerPosDelta);
//                telemetryPipeline.addDataPoint("tolerance", cocker.getTargetPositionTolerance());
//                break;
//            case 1:
//                cocker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                cocker.setPower(HardwareConstants.CONVEYOR_POWER);
//
//                if (doubleDeltaOnPress.check(secondaryCtrl)) {
//                    conveyorPowerDelta *= 2;
//                } else if (halveDeltaOnPress.check(secondaryCtrl)) {
//                    conveyorPowerDelta /= 2;
//                }
//
//                if (incrementOnPress.check(primaryCtrl)) {
//                    HardwareConstants.CONVEYOR_POWER += conveyorPowerDelta;
//                } else if (decrementOnPress.check(primaryCtrl)) {
//                    HardwareConstants.CONVEYOR_POWER -= conveyorPowerDelta;
//                }
//
//                telemetryPipeline.addDataPoint("MODE", "precise power");
//                telemetryPipeline.addDataPoint("cocker power delta", conveyorPowerDelta);
//                telemetryPipeline.addDataPoint("intended cocker power", HardwareConstants.CONVEYOR_POWER);
//                break;
//            case 2:
//                cocker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                double power = secondaryCtrl.getLeftX();
//                power *= Math.abs(power);
//                cocker.setPower(power);
//
//                if (doubleDeltaOnPress.check(secondaryCtrl)) {
//                    cockerPosDelta *= 2;
//                } else if (halveDeltaOnPress.check(secondaryCtrl)) {
//                    cockerPosDelta /= 2;
//                }
//
//                if (incrementOnPress.check(primaryCtrl)) {
//                    HardwareConstants.CONVEYOR_POS += cockerPosDelta;
//                } else if (decrementOnPress.check(primaryCtrl)) {
//                    HardwareConstants.CONVEYOR_POS -= cockerPosDelta;
//                }
//
//                telemetryPipeline.addDataPoint("MODE", "stick power");
//                telemetryPipeline.addDataPoint("input cocker power", power);
//                break;
//            case -1:
//                telemetryPipeline.addDataPoint("MODE", "mode not set");
//                telemetryPipeline.addHeader("A, B & Y on player 2 set the mode");
//                telemetryPipeline.addDataPoint("MODE", "mode not set");
//        }

        telemetryPipeline.refresh();
    }
}