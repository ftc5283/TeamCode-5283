package org.firstinspires.ftc.teamcode.actions;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.ReferenceInt;
import org.firstinspires.ftc.teamcode.pipeline.TelemetryPipeline;
import org.firstinspires.ftc.teamcode.utility.ActionWrapper;
import org.firstinspires.ftc.teamcode.utility.Status;
import org.firstinspires.ftc.teamcode.utility.Task;

public class MotorActions {
    final public DcMotorEx motor;
    public final String motorName;
    private MoveMotor moveMotor;
    public final WaitMotor waitMotor = new WaitMotor();
    public final MotorActionTelemetry telemetry;
    private Task moveMotorTask;
    private Task waitMotorTask;
    public final ClearPerpetualTelemTask clearTelemTask = new ClearPerpetualTelemTask();

    public MotorActions(@NonNull DcMotorEx motor, @NonNull TelemetryPipeline telemetryPipeline){
        this.motor = motor;
        this.motorName = motor.getDeviceName();
        this.telemetry = new MotorActionTelemetry(telemetryPipeline);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public MotorActions(@NonNull DcMotorEx motor, @NonNull String motorName, @NonNull TelemetryPipeline telemetryPipeline){
        this.motor = motor;
        this.motorName = motorName;
        this.telemetry = new MotorActionTelemetry(telemetryPipeline);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public MotorActions(@NonNull HardwareMap hardwareMap, @NonNull String motorName, @NonNull TelemetryPipeline telemetryPipeline){
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);
        this.motorName = motorName;
        this.telemetry = new MotorActionTelemetry(telemetryPipeline);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public MotorActions(@NonNull HardwareMap hardwareMap, @NonNull String motorName, @NonNull String deviceName, @NonNull TelemetryPipeline telemetryPipeline){
        this.motor = hardwareMap.get(DcMotorEx.class, deviceName);
        this.motorName = motorName;
        this.telemetry = new MotorActionTelemetry(telemetryPipeline);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public class MoveMotor implements Action {
        public int getTarget(){
            return targetPosRef == null ? targetPos : targetPosRef.value;
        }
        public int getTolerance(){
            return toleranceRef == null ? tolerance : toleranceRef.value;
        }
        public int getDamping(){
            return dampingRef == null ? damping : dampingRef.value;
        }
        public int targetPos, tolerance, damping;
        protected ReferenceInt targetPosRef, toleranceRef, dampingRef;
        public final ElapsedTime lastRun;
        public double powerMultiplier;
        MoveMotor(int targetPosition, int tolerance, int damping){
            this(targetPosition, tolerance, damping, 1);
        }
        MoveMotor(int targetPosition, int tolerance, int damping, double powerMultiplier){

            this.targetPos = targetPosition;
            this.targetPosRef = null;
            this.tolerance = tolerance;
            this.damping = damping;
            this.powerMultiplier = powerMultiplier;
            lastRun = new ElapsedTime();
        }

        MoveMotor(@NonNull ReferenceInt targetPosition, @NonNull ReferenceInt tolerance, @NonNull ReferenceInt damping){
            this.targetPosRef = targetPosition;
            this.toleranceRef = tolerance;
            this.dampingRef = damping;

            this.targetPos = targetPosition.value;
            this.tolerance = tolerance.value;
            this.damping = damping.value;
            lastRun = new ElapsedTime();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return run();
        }

        public boolean run() {
            lastRun.reset();
            int pos = motor.getCurrentPosition();
            int targetPosition = getTarget();
            tolerance = getTolerance();
            damping = getDamping();
            if(pos > targetPosition + tolerance) {
                if(pos < targetPosition + damping)
                    motor.setPower(powerMultiplier * ((double) (targetPosition - pos) / damping));
                else
                    motor.setPower(-powerMultiplier);
                return true;
            } else if(pos + tolerance < targetPosition) {
                if(pos + damping > targetPosition)
                    motor.setPower(powerMultiplier * ((double) (targetPosition - pos) / damping));
                else
                    motor.setPower(powerMultiplier);
                return true;
            } else {
                motor.setPower(0);
                return false;
            }
        }

        public double getPower(){
            return motor.getPower();
        }

        public boolean within(){
            return within(tolerance);
        }

        public boolean within(int tolerance){
            int pos = motor.getCurrentPosition();
            int targetPosition = getTarget();
            if(pos > targetPosition + tolerance || pos + tolerance < targetPosition) {
                return false;
            } else {
                return true;
            }
        }

        public boolean isSlow(){
            int pos = motor.getCurrentPosition();
            int targetPosition = getTarget();
            int slowTolerance = Math.max(damping/2, this.tolerance);
            return pos <= targetPosition + slowTolerance && pos + slowTolerance >= targetPosition;
        }

        public boolean isSlow(double tolerance){
            int pos = motor.getCurrentPosition();
            int targetPosition = getTarget();
            double slowTolerance = Math.max(damping*tolerance, this.tolerance);
            return pos <= targetPosition + slowTolerance && pos + slowTolerance >= targetPosition;
        }

        public boolean isDampened(){
            int pos = motor.getCurrentPosition();
            int targetPosition = getTarget();
            int tolerance = Math.max(damping, this.tolerance);
            return pos <= targetPosition + tolerance && pos + tolerance >= targetPosition;
        }
        public Task taskPerpetual() {
            return getMoveMotorTaskPerpetual();
        }

    }

    public MoveMotor moveMotor (int targetPosition, int tolerance, int damping){
        if(moveMotor == null){
            moveMotor = new MoveMotor(targetPosition, tolerance, damping);
        } else {
            moveMotor.targetPosRef = null;
            moveMotor.toleranceRef = null;
            moveMotor.dampingRef = null;

            moveMotor.targetPos = targetPosition;
            moveMotor.tolerance = tolerance;
            moveMotor.damping = damping;
        }
        return new MoveMotor(targetPosition, tolerance, damping);
    }
    public MoveMotor moveMotor (@NonNull ReferenceInt targetPosition, @NonNull ReferenceInt  tolerance, @NonNull ReferenceInt  damping){
        if(moveMotor == null){
            moveMotor = new MoveMotor(targetPosition, tolerance, damping);
        } else {
            moveMotor.targetPosRef = targetPosition;
            moveMotor.toleranceRef = tolerance;
            moveMotor.dampingRef = damping;
        }
        return moveMotor;
    }
    public MoveMotor moveMotor (int targetPosition){
        if(moveMotor == null) {
            moveMotor = new MoveMotor(targetPosition, 10, 100);
        } else {
            moveMotor.targetPos = targetPosition;
            moveMotor.targetPosRef = null;
            moveMotor.toleranceRef = null;
            moveMotor.dampingRef = null;
        }
        return moveMotor;
    }
    public MoveMotor moveMotor(ReferenceInt targetPosition) {
        if (moveMotor == null){
            moveMotor = new MoveMotor(targetPosition, new ReferenceInt(10), new ReferenceInt(100));
        } else {
            moveMotor.targetPosRef = targetPosition;
        }
        return moveMotor;
    }

    public class MotorActionTelemetry implements Task {
        public final TelemetryPipeline telemetry;
        public final ElapsedTime lastRun;
        public final String posKey = motorName+" Pos:";
        public final String targetKey = motorName+" Target:";
        public final String slowUpperKey = motorName+" Slow Upper:";
        public final String slowLowerKey = motorName+" Slow Lower:";
        public final String slowKey = motorName+" slow?:";
        public final String dampenedKey = motorName+" dampened?:";

        MotorActionTelemetry(@NonNull TelemetryPipeline telemetryPipeline){
            this.telemetry = telemetryPipeline;
            lastRun = new ElapsedTime();
        }

        @Override
        public Status run() {
            int pos = motor.getCurrentPosition();
            telemetry.addDataPointPerpetual(posKey, pos);
            if(moveMotor != null){
                int target = moveMotor.getTarget();
                double slowTolerance = Math.max(moveMotor.tolerance, moveMotor.damping * waitMotor.tolerance);
                telemetry.addDataPointPerpetual(targetKey, target);
                telemetry.addDataPointPerpetual(slowUpperKey, target + slowTolerance);
                telemetry.addDataPointPerpetual(slowLowerKey, target - slowTolerance);
                telemetry.addDataPointPerpetual(slowKey, moveMotor.isSlow());
                telemetry.addDataPointPerpetual(dampenedKey, moveMotor.isDampened());
            }
            return Status.RUNNING;
        }
        public boolean clearPerpetualTelem(){
            return telemetry.removeDataPoint(posKey) & (
                    moveMotor == null ||
                            (telemetry.removeDataPoint(targetKey) &
                            telemetry.removeDataPoint(slowUpperKey) &
                            telemetry.removeDataPoint(slowLowerKey) &
                            telemetry.removeDataPoint(slowKey) &
                            telemetry.removeDataPoint(dampenedKey))
                    );
        }
    }

    public class ClearPerpetualTelemTask implements Task {
        ClearPerpetualTelemTask(){}

        @Override
        public Status run() {
            telemetry.clearPerpetualTelem();
            return Status.SUCCESS;
        }
    }


    public void stop() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public DcMotorEx getMotor () {
        return motor;
    }

    public class WaitMotor implements Action {
        public double tolerance;
        WaitMotor() {
            tolerance = 0.05;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return Math.abs(moveMotor.getPower()) > tolerance;
        }

        public Task task(){
            return getWaitMotorTask();
        }
    }

    public WaitMotor waitMotor (double tolerance) {
        waitMotor.tolerance = tolerance;
        return waitMotor;
    }
    public MoveMotor getMoveMotor() {
        return moveMotor;
    }

    public Task getMoveMotorTaskPerpetual(){
        if(moveMotorTask == null)
            moveMotorTask = ActionWrapper.toPerpetualTask(moveMotor, "Move "+motorName, telemetry);
        return moveMotorTask;
    }
    public Task getMoveMotorTaskPerpetual(double speedMult){
        moveMotor.powerMultiplier = speedMult;
        if(moveMotorTask == null)
            moveMotorTask = ActionWrapper.toPerpetualTask(moveMotor, "Move "+motorName, telemetry);
        return moveMotorTask;
    }
    public Task getWaitMotorTask(){
        if(waitMotorTask == null) {
            waitMotorTask = ActionWrapper.toTask(waitMotor, "Wait "+motorName, telemetry);
        }

        return waitMotorTask;
    }
    public boolean slow(){
        return moveMotor.isSlow();
    }
    public boolean slow(double tolerance){
        return moveMotor.isSlow(tolerance);
    }
}
