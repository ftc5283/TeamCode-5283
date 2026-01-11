package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    CRServoImplEx fwl, fwr, intake;
    ServoImplEx kicker, floor;
    DcMotorEx flyWheel;
    VoltageSensor controlHub;

    public void setMiniFlyWheels(double power) {
        fwr.setPower(-power);
        fwl.setPower(power);
    }

    public void kickAndShoot(){
        this.setMiniFlyWheels(0);
        intake.setPower(1);

        sleep(500);
        kicker.setPosition(HardwareConstants.KICKER_KICK_POS);
        sleep(1000);

        this.setMiniFlyWheels(1);

        sleep(500);
        kicker.setPosition(0);
        sleep(500);

        this.setMiniFlyWheels(0);
    }

    public void initialize(boolean startParallelTelemetry) {
        telemetryPipeline = new TelemetryPipeline(telemetry);
        drive = new HardwarePipeline(hardwareMap);

        timer = new ElapsedTime();

        intake = hardwareMap.get(CRServoImplEx.class, "intake");
        fwr = hardwareMap.get(CRServoImplEx.class, "fwr");
        fwl = hardwareMap.get(CRServoImplEx.class, "fwl");
        kicker = hardwareMap.get(ServoImplEx.class, "kicker");
        floor = hardwareMap.get(ServoImplEx.class, "floor");

        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        controlHub = hardwareMap.voltageSensor.get("Control Hub");

        floor.setPosition(0);

        if (startParallelTelemetry) {
            this.startParallelTelemetry();
        }
    }

    public void shoot() {
        flyWheel.setVelocity(HardwareConstants.FLY_WHEEL_VEL, AngleUnit.RADIANS);

        sleep(7000);

        // using `this` instead of just calling the method so that its more visually distinct from sleep
        // even though sleep is also an instance method.
        this.setMiniFlyWheels(1);

        sleep(1000);

        this.kickAndShoot();

        sleep(1000);

        floor.setPosition(HardwareConstants.FLOOR_POS);

        sleep(1000);

        this.kickAndShoot();

        sleep(1000);

        intake.setPower(0);
    }

    public void leak() {
        //just in case
        kicker.setPosition(0);

        // ball 1
        intake.setPower(-1);
        sleep(2000);

        // ball 2
        this.setMiniFlyWheels(-1);
        sleep(2000);

        // ball 3
        floor.setPosition(HardwareConstants.FLOOR_POS);
        sleep(2000);

        // reset
        intake.setPower(0);
        this.setMiniFlyWheels(0);
        floor.setPosition(0);
    }

    public void stopFlyWheelNice(){
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheel.setPower(0);
        sleep(3000);
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheel.setPower(0.01);
        flyWheel.setPower(0);
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
                    telemetryPipeline.addDataPointPerpetual("flyWheel power", flyWheel.getPower());
                    telemetryPipeline.addDataPointPerpetual("flyWheel current", flyWheel.getCurrent(CurrentUnit.MILLIAMPS));
                    telemetryPipeline.addDataPointPerpetual("flyWheel velocity", flyWheel.getVelocity(AngleUnit.RADIANS));
                    telemetryPipeline.refresh();
                }
            }
        };
        telemetryThread.start();
    }
}