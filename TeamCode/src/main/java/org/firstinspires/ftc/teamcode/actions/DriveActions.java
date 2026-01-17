package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pipeline.HardwarePipeline;
import org.firstinspires.ftc.teamcode.pipeline.TelemetryPipeline;
import org.firstinspires.ftc.teamcode.utility.Status;
import org.firstinspires.ftc.teamcode.utility.Task;

import static java.lang.Math.abs;
public class DriveActions {
    final public HardwarePipeline hardwarePipeline;
    private HardwareMap hardwareMap;
    final public DcMotorEx par0;
    final public DcMotorEx par1;
    final public DcMotorEx perp;
    final public TelemetryPipeline telemetryPipeline;

    public DriveActions(HardwarePipeline hardwarePipeline, HardwareMap hardwareMap, TelemetryPipeline telemetryPipeline) {
        this.hardwarePipeline = hardwarePipeline;
        this.hardwareMap = hardwareMap;
        this.telemetryPipeline = telemetryPipeline;

        par0 = hardwareMap.get(DcMotorEx.class, "dcBr");//right - port 0
        par1 = hardwareMap.get(DcMotorEx.class, "dcFr");//left - port 3
        perp = hardwareMap.get(DcMotorEx.class, "dcBl");//center - port 2
    }

    public static void sleepSec(double sec){
        ElapsedTime time = new ElapsedTime();
        while(time.seconds() <= sec);
    }

    public class SleepTask implements Task {
        public final double duration;
        public ElapsedTime timer;
        public SleepTask(double duration){
            this.duration = duration;
        }
        public void reset(){
            if(timer == null)
                timer = new ElapsedTime();
            else
                timer.reset();
        }
        @Override
        public Status run() {
            return null;
        }
    }
    public DriveActions(HardwarePipeline hardwarePipeline, DcMotorEx par0, DcMotorEx par1, DcMotorEx perp, TelemetryPipeline telemetryPipeline) {
        this.hardwarePipeline = hardwarePipeline;
        this.par0 = par0;//right - port 0
        this.par1 = par1;//left - port 3
        this.perp = perp;//center - port 2
        this.telemetryPipeline = telemetryPipeline;
    }

    public ToWall toWall() {
        return new ToWall(10, 30, 100, -0.8, 0);
    }

    public ToWall toWall(int storeLength, int tolerance) {
        return new ToWall(storeLength, tolerance, tolerance, 1, 0);
    }

    public ToWall toWall(int storeLength, int startTolerance, int endTolerance) {
        return new ToWall(storeLength, startTolerance, endTolerance, -0.5, 0);
    }

    public ToWall toWall(double forwardSpeed, double strafeSpeed) {
        return new ToWall(10, 30,100, forwardSpeed, strafeSpeed);
    }

    public ToWall toWall(int storeLength, int tolerance, double forwardSpeed, double strafeSpeed) {
        return new ToWall(storeLength, tolerance, tolerance, forwardSpeed, strafeSpeed);
    }

    public ToWall toWall(int storeLength, int startTolerance, int endTolerance, double forwardSpeed, double strafeSpeed) {
        return new ToWall(storeLength, startTolerance, endTolerance, forwardSpeed, strafeSpeed);
    }

    public class ToWall implements Action {
        final public String fowardKey = "Forward Speed",
                            strafeKey = "Strafe Speed",
                            par0Key = "Par0 Dif",
                            par1Key = "Par1 Dif",
                            perpKey = "Perp Dif";
        boolean starting = true;

        final public int[] par0Readings;
        final public int[] par1Readings;
        final public int[] perpReadings;

        final public int endTolerance;
        final public int startTolerance;

        final public double strafeSpeed;
        final public double forwardSpeed;
        int curReading = 0;

        public ToWall(int storeLength, int startTolerance, int endTolerance, double forwardSpeed, double strafeSpeed) {

            this.par0Readings = new int[storeLength];
            this.par1Readings = new int[storeLength];
            this.perpReadings = new int[storeLength];

            this.startTolerance = startTolerance +
                    Math.max(
                            Math.abs(par0.getCurrentPosition()),
                            Math.max(
                                    Math.abs(par1.getCurrentPosition()),
                                    Math.abs(perp.getCurrentPosition())
                            )
                    );
            this.endTolerance = endTolerance;

            this.forwardSpeed = forwardSpeed;
            this.strafeSpeed = strafeSpeed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return run();
        }

        public int[] getDifs(){
            if (starting &&
                    (abs(par0Readings[curReading]) > startTolerance
                            || abs(par1Readings[curReading]) > startTolerance
                            || abs(perpReadings[curReading]) > startTolerance)) {
                starting = false;
            }

            par0Readings[curReading] = par0.getCurrentPosition();
            par1Readings[curReading] = par1.getCurrentPosition();
            perpReadings[curReading] = perp.getCurrentPosition();

            int oldestReading = (curReading + 1) % par0Readings.length;

            int par0Dif = abs(par0Readings[oldestReading] - par0Readings[curReading]);
            int par1Dif = abs(par1Readings[oldestReading] - par1Readings[curReading]);
            int perpDif = abs(perpReadings[oldestReading] - perpReadings[curReading]);

            curReading = oldestReading;
            return new int[]{par0Dif, par1Dif, perpDif};
        }

        public boolean run(int par0Dif,int  par1Dif, int perpDif){
            if (starting || par0Dif > endTolerance || par1Dif > endTolerance || perpDif > endTolerance) {
                hardwarePipeline.driveRobotCentric(strafeSpeed, forwardSpeed, 0);
                return true;
            } else {
                hardwarePipeline.setPowerBehavior();
                hardwarePipeline.stopAll();
                return false;
            }
        }

        public boolean run(@NonNull int[] difs){
            return run(difs[0], difs[1], difs[2]);
        }

        public boolean run() {
            return run(getDifs());
        }

        public Task toTask(){
            ToWall this_ = this;
            return new Task() {
                @Override
                public Status run() {
                    int[] difs = getDifs();
                    telemetryPipeline.addDataPointPerpetual(fowardKey, forwardSpeed);
                    telemetryPipeline.addDataPointPerpetual(strafeKey, strafeSpeed);
                    telemetryPipeline.addDataPointPerpetual(par0Key, difs[0]);
                    telemetryPipeline.addDataPointPerpetual(par1Key, difs[1]);
                    telemetryPipeline.addDataPointPerpetual(perpKey, difs[2]);
                    return this_.run(difs) ? Status.RUNNING : Status.SUCCESS;
                }
            };
        }
        public boolean clearTelemtry(){
            return  telemetryPipeline.removeDataPoint(fowardKey) &
                    telemetryPipeline.removeDataPoint(strafeKey) &
                    telemetryPipeline.removeDataPoint(par0Key) &
                    telemetryPipeline.removeDataPoint(par1Key) &
                    telemetryPipeline.removeDataPoint(perpKey);
        }
    }



}
