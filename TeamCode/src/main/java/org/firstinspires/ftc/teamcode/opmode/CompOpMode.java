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

        primaryCtrl = gamepadEx1;
        secondaryCtrl = gamepadEx2;
        /*
           this might be the one time this is a bad idea, this might stress the motor a lot lol.
         */
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // PRIMARY CONTROLLER
    final ButtonToggle intakeXToggle = new ButtonToggle(X, false);
    final ButtonToggle flyWheelRBumperToggle = new ButtonToggle(RIGHT_BUMPER, false);
    final ButtonToggle kickerBToggle = new ButtonToggle(B, false);
    final ButtonToggle throttleLeftStickToggle = new ButtonToggle(LEFT_STICK_BUTTON, true);
    final ButtonOnPress floorYOnPress = new ButtonOnPress(Y);

    // SECONDARY CONTROLLER
    final ButtonToggle holdFloorYToggle = new ButtonToggle(Y, false);
    final ButtonToggle testRightStickToggle = new ButtonToggle(RIGHT_STICK_BUTTON, false);

    @SuppressWarnings("unused")

    final ButtonOnPress incrementOnPress = new ButtonOnPress(DPAD_UP);
    @SuppressWarnings("unused")
    final ButtonOnPress decrementOnPress = new ButtonOnPress(DPAD_DOWN);

    GamepadEx primaryCtrl, secondaryCtrl;
    double flyWheelCurrent = 0;

    @FunctionalInterface
    private interface Double2Double {double map(double x);}
    final boolean squareInputs = false;
    final Double2Double squareInputsCorrection = (squareInputs ? (double x) -> x : Math::sqrt);
    final double throttleStrafe = squareInputsCorrection.map(0.5);
    final double throttleForwardBack = squareInputsCorrection.map(0.5);
    final double throttleTurn = squareInputsCorrection.map(0.5);

    public void setMiniFlyWheelPowers(double speed) {
        fwr.setPower(-speed);
        fwl.setPower(speed);
    }

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

//        if (incrementOnPress.check(primaryCtrl)) {
//            var += 0.01;
//        } else if (decrementOnPress.check(primaryCtrl)) {
//            var -= 0.01;
//        }

        if (secondaryCtrl.getButton(X)) {
            setMiniFlyWheelPowers(-1);
            intake.setPower(-1);
            kicker.setPosition(0);
        } else {
            setMiniFlyWheelPowers(primaryCtrl.getButton(LEFT_BUMPER) ? 1 : 0);

            intake.setPower(intakeXToggle.check(primaryCtrl) ? 1 : 0);

            kicker.setPosition(kickerBToggle.check(primaryCtrl) ? HardwareConstants.KICKER_KICK_POS : 0);

            if (
                floorYOnPress.checkWithin(primaryCtrl, 1000) ||
                holdFloorYToggle.check(secondaryCtrl)
            ) {
                floor.setPosition(HardwareConstants.FLOOR_POS);
            } else {
                floor.setPosition(0);
            }
        }



        double turnSpeed = gamepad1.right_trigger - gamepad1.left_trigger;
        double strafeSpeed = gamepadEx1.getLeftX();
        double forwardSpeed = gamepadEx1.getLeftY();
        boolean throttleSpeed = throttleLeftStickToggle.check(primaryCtrl);

        drive.driveRobotCentric(
                -strafeSpeed * (throttleSpeed ? throttleStrafe : 1),
                -forwardSpeed * (throttleSpeed ? throttleForwardBack : 1),
                -turnSpeed * (throttleSpeed ? throttleTurn : 1),
                true
        );

        supervisor.run(telemetryPipeline);

        flyWheel.setVelocity(flyWheelRBumperToggle.check(primaryCtrl) ? HardwareConstants.FLY_WHEEL_VEL : 0, AngleUnit.RADIANS);

        flyWheelCurrent = flyWheel.getCurrent(CurrentUnit.MILLIAMPS);

        telemetryPipeline.addDataPoint("forward Speed", forwardSpeed);
        telemetryPipeline.addDataPoint("turn Speed", turnSpeed);
        telemetryPipeline.addDataPoint("strafe Speed", strafeSpeed);
        telemetryPipeline.addDataPoint("FlyWheel Current (mA)", flyWheelCurrent);
        telemetryPipeline.addDataPoint("FlyWheel Power", flyWheel.getPower());
        telemetryPipeline.addDataPoint("FlyWheel Velocity", flyWheel.getVelocity(AngleUnit.RADIANS));
        telemetryPipeline.addDataPoint("FLY_WHEEL_VEL", HardwareConstants.FLY_WHEEL_VEL);
        telemetryPipeline.refresh();
    }
}