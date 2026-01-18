package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pipeline.HardwarePipeline;
import org.firstinspires.ftc.teamcode.pipeline.TelemetryPipeline;
import org.firstinspires.ftc.teamcode.utility.HardwareConstants;
import org.firstinspires.ftc.teamcode.utility.SimpleAtomicBool;
import static org.firstinspires.ftc.teamcode.utility.Misc.withinTolerance;

@Autonomous(name = "Reset Motors", group = "Main")
public class ResetMotorsPos extends LinearOpMode {
    protected TelemetryPipeline telemetryPipeline;
    protected HardwarePipeline drive;

    ElapsedTime timer;

    DcMotorEx cocker, conveyor;
    VoltageSensor controlHub;

    int cockerInitPos;
    int conveyorInitPos;

    final SimpleAtomicBool cockerComplete = new SimpleAtomicBool();
    final SimpleAtomicBool conveyorComplete = new SimpleAtomicBool();

    final static int timeRes = 100;
    final static int measures = 5;

    final static double conveyorSpeedEncodePerMillis = 30.0/500;
    final static double cockerSpeedEncodePerMillis = 35.0/500;

    public static void motorRunTillCollision(
        String name,
        DcMotorEx motor,
        double power,
        TelemetryPipeline telemetry,
        int posTolerance,
        int measurements,
        int timeResolutionMillis
    ) {
        motor.setPower(power);

        int[] readings = new int[measurements];
        for (int i = 0; i < measurements; i++) {
            readings[i] = motor.getCurrentPosition();
            try {
                Thread.sleep(timeResolutionMillis);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        int i = measurements-1;
        int old;
        String reading = name+" reading";
        String oldReading = "old "+reading;
//        ElapsedTime timer = new ElapsedTime();
        do {
            i = (i+1)%measurements;
            old = readings[i];
            try {
                //noinspection BusyWait//
                Thread.sleep(timeResolutionMillis);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            readings[i] = motor.getCurrentPosition();
            telemetry.addDataPointPerpetual(oldReading, old);
            telemetry.addDataPointPerpetual(reading, readings[i]);
        } while (!withinTolerance(old, readings[i], posTolerance));
        telemetry.removeDataPoint(reading);
        telemetry.removeDataPoint(oldReading);
    }

    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetryPipeline = new TelemetryPipeline(telemetry);
        drive = new HardwarePipeline(hardwareMap);

        cocker = hardwareMap.get(DcMotorEx.class, "cocker");
        cocker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cockerInitPos = cocker.getCurrentPosition();
        telemetryPipeline.addDataPointPerpetual("Cocker init pos", cockerInitPos);

        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");
        conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        conveyorInitPos = conveyor.getCurrentPosition();
        telemetryPipeline.addDataPointPerpetual("Conveyor init pos", conveyorInitPos);

        controlHub = hardwareMap.voltageSensor.get("Control Hub");

        timer = new ElapsedTime();

        telemetryPipeline.addHeader("Waiting to start");
        telemetryPipeline.refresh();

        Thread cockerThread = new Thread(() -> {
            int tolerance = (int) (cockerSpeedEncodePerMillis * timeRes * measures);
            motorRunTillCollision(
                "Cocker",
                cocker,
                HardwareConstants.COCKER_WEAK_POWER,
                telemetryPipeline,
                tolerance,
                measures,
                timeRes
            );
            cockerComplete.bool = true;
        });


        Thread conveyorThread = new Thread(() -> {
            motorRunTillCollision(
                "Conveyor",
                conveyor,
                HardwareConstants.CONVEYOR_WEAK_POWER,
                telemetryPipeline,
                (int) (conveyorSpeedEncodePerMillis * timeRes * measures),
                measures,
                timeRes
            );
            conveyorComplete.bool = true;
        });

        waitForStart();

        conveyorThread.start();
        cockerThread.start();

        while (!cockerComplete.bool || !conveyorComplete.bool) {
            telemetryPipeline.addDataPoint("Cocker pos", cocker.getCurrentPosition());
            telemetryPipeline.addDataPoint("Conveyor pos", conveyor.getCurrentPosition());
            if (cockerComplete.bool) {
                telemetryPipeline.addHeaderPerpetual("cocker reset pos complete");
            }
            if (conveyorComplete.bool) {
                telemetryPipeline.addHeaderPerpetual("cocker reset pos complete");
            }
            telemetryPipeline.refresh();
        }

        telemetryPipeline.addHeader("PROCESS COMPLETE.");
        telemetryPipeline.addHeader("WILL AUTOMATICALLY TURN OFF IN 10s");
        telemetryPipeline.addHeader("LAST CHANCE TO VIEW TELEMETRY");

        telemetryPipeline.addDataPoint("last cocker pos before reset", cocker.getCurrentPosition());
        telemetryPipeline.addDataPoint("last conveyor pos before rest", conveyor.getCurrentPosition());
        telemetryPipeline.addDataPoint("init cocker pos", cockerInitPos);
        telemetryPipeline.addDataPoint("init conveyor pos", conveyorInitPos);

        cocker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
        telemetryPipeline.addDataPoint("Cocker pos (should be ~0)", cocker.getCurrentPosition());
        telemetryPipeline.addDataPoint("Conveyor pos (should be ~0)", conveyor.getCurrentPosition());

        if (cockerComplete.bool) {
            telemetryPipeline.addHeaderPerpetual("cocker reset pos complete");
        }
        if (conveyorComplete.bool) {
            telemetryPipeline.addHeaderPerpetual("cocker reset pos complete");
        }

        telemetryPipeline.refresh();

        sleep(10000);
    }
}
