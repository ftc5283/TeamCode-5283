package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pipeline.HardwarePipeline;
import org.firstinspires.ftc.teamcode.pipeline.TelemetryPipeline;
import org.firstinspires.ftc.teamcode.utility.HardwareConstants;
import org.firstinspires.ftc.teamcode.utility.Supervisor;

public abstract class AutoSuperClass extends LinearOpMode {

    protected TelemetryPipeline telemetryPipeline;
    protected HardwarePipeline drive;
    protected final Supervisor supervisor = new Supervisor();

    ElapsedTime timer;

    DcMotorEx cocker;
    VoltageSensor controlHub;

    public void loadAndShoot() {

    }

    public static DcMotorEx getCocker(HardwareMap hardwareMap) {
        DcMotorEx cocker = hardwareMap.get(DcMotorEx.class, "cocker");
        cocker.setTargetPosition(0);
        cocker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cocker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return cocker;
    }

    public void initialize(boolean startParallelTelemetry) {
        telemetryPipeline = new TelemetryPipeline(telemetry);
        drive = new HardwarePipeline(hardwareMap);

        timer = new ElapsedTime();

        cocker = getCocker(hardwareMap);

        controlHub = hardwareMap.voltageSensor.get("Control Hub");

        if (startParallelTelemetry) {
            this.startParallelTelemetry();
        }
    }

    public void shoot1() {
//        cocker.setTargetPosition(HardwareConstants.COCKER_POS)
        cocker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(5000);
        cocker.setPower(0.7);
        sleep(5000);
        cocker.setPower(0);
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
        LinearOpMode auto = this;
        Thread telemetryThread = new Thread() {
            @Override
            public void run(){
                while (!auto.isStopRequested()) {
                    telemetryPipeline.addDataPointPerpetual("real cocker pos", cocker.getCurrentPosition());
                    telemetryPipeline.addDataPointPerpetual("real cocker target", cocker.getTargetPosition());
                    telemetryPipeline.addDataPointPerpetual("set cocker target", HardwareConstants.CONVEYOR_POS);
                    telemetryPipeline.addDataPoint("cocker current (mA)", cocker.getCurrent(CurrentUnit.MILLIAMPS));
                    telemetryPipeline.addDataPoint("max safe current (mA)", cocker.getCurrentAlert(CurrentUnit.MILLIAMPS));
                    telemetryPipeline.refresh();
                }
            }
        };
        telemetryThread.start();
    }
}