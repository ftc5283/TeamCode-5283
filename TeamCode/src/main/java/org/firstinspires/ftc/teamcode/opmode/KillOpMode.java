package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.utility.Misc.cast;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pipeline.HardwarePipeline;
import org.firstinspires.ftc.teamcode.pipeline.TelemetryPipeline;
import org.firstinspires.ftc.teamcode.pipeline.WheelDirection;
import org.firstinspires.ftc.teamcode.utility.ButtonOnPress;
import org.firstinspires.ftc.teamcode.utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.utility.Supervisor;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.*;
import static java.lang.Math.abs;
import static java.lang.Math.log;
import static java.lang.Math.signum;

import java.text.DecimalFormat;

@TeleOp(name = "Dont Run This. Will Destroy Robot")
public class KillOpMode extends OpMode{
    protected TelemetryPipeline telemetryPipeline;
    protected HardwarePipeline drive;
    CRServoImplEx fwl, fwr, intake;
    DcMotorEx flyWheel;

    @Override
    public void init() {
        telemetryPipeline = new TelemetryPipeline(telemetry);
        drive = new HardwarePipeline(hardwareMap);
        intake = hardwareMap.get(CRServoImplEx.class, "intake");
        fwr = hardwareMap.get(CRServoImplEx.class, "fwr");
        fwl = hardwareMap.get(CRServoImplEx.class, "fwl");
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
    }
    private  double randomPower() {
        return Math.random() * (Math.random() > 0.5 ? 1 : -1);
    }
    @Override
    public void loop() {
        drive.driveWithMotorPowers(randomPower(), randomPower(), randomPower(), randomPower());
        flyWheel.setPower(randomPower());
        fwr.setPower(randomPower());
        fwl.setPower(randomPower());
        intake.setPower(randomPower());
        telemetryPipeline.addDataPoint("Reminder", "you chose this");
        telemetryPipeline.addHeader("why");
        telemetryPipeline.refresh();
    }
}