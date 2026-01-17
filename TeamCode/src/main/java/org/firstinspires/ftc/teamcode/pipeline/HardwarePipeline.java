package org.firstinspires.ftc.teamcode.pipeline;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.pipeline.WheelDirection.*;

public class HardwarePipeline extends MecanumDrive {
    private final MotorGroup dcDrive;

    public HardwarePipeline(HardwareMap hardwareMap) {
        this(
            new Motor(hardwareMap, "dcFl"),
            new Motor(hardwareMap, "dcFr"),
            new Motor(hardwareMap, "dcBl"),
            new Motor(hardwareMap, "dcBr")
        );
    }
    public HardwarePipeline(Motor dcFl, Motor dcFr, Motor dcBl, Motor dcBr) {
        //noinspection ConstantValue
        super(INVERT_LEFT_SIDE ^ INVERT_RIGHT_SIDE, dcFl, dcFr, dcBl, dcBr);
        dcFr.setInverted(!dcFr.getInverted());
        dcDrive = new MotorGroup(dcFl, dcFr, dcBl, dcBr);
        setPowerBehavior();
    }

    public void setPowerBehavior() {
        dcDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void resetEncoders() {
        dcDrive.resetEncoder();
    }

    public void setRunMode(Motor.RunMode mode) {
        dcDrive.setRunMode(mode);
    }

    public void stopAll() {
        dcDrive.stopMotor();
    }

    @Override
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        this.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed, false);
    }

    @Override
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, boolean squareInputs) {
        if(INVERT_LEFT_SIDE) {
            super.driveRobotCentric(-strafeSpeed, -forwardSpeed, -turnSpeed, squareInputs);
        } else {
            super.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed,squareInputs);
        }
    }
}
