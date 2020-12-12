package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.util.Constants.motorTicksPerRev;

public class WobbleMech {

    public DcMotorEx motor;
    public Servo servo;

    private final double WOBBLE_TICKS_PER_REV = motorTicksPerRev[2];
    private final double WOBBLE_TICKS_PER_DEGREE = WOBBLE_TICKS_PER_REV/360;
    private final double[] wobbleAngleTicks =
            {115 * WOBBLE_TICKS_PER_DEGREE,
                    0 * WOBBLE_TICKS_PER_DEGREE,
                    -90 * WOBBLE_TICKS_PER_DEGREE}; // temp

    private final double sOpen = 0.0; // temp
    private final double sClose = 0.35; // temp

    public WobbleMech() {

    }

    public void initialize() {
        this.motor.setDirection(DcMotor.Direction.REVERSE);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.setMotorPosition(1);

        this.close();
    }

    public void setMotorPosition(int position) {
        this.motor.setTargetPosition((int)Math.round(wobbleAngleTicks[position]));
        this.setPower(0.4);
        if (this.getMotorPosition() < 3 && this.getMotorPosition() > -3) {
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public int getMotorPosition() {
        return this.motor.getCurrentPosition();
    }

    public void setPower(double power) {
        this.motor.setPower(power);
    }

    public void open() {
        this.servo.setPosition(sOpen);
    }

    public void close() {
        this.servo.setPosition(sClose);
    }

    public double getServoPosition() {
        return this.servo.getPosition();
    }
}
