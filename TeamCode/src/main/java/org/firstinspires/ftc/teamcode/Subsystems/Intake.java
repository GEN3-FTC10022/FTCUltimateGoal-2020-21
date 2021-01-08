package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.Util.Constants.motorTicksPerRev;

public class Intake {

    // Objects
    public DcMotorEx rollers;

    // Constants
    public Status status;

    public Intake() { }

    public enum Status {
        IN,
        OUT,
        OFF
    }

    public void initialize() {

        // Arm
        rollers.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // motor.setDirection(DcMotorSimple.Direction.REVERSE);
        off();
    }

    public void in() {
        rollers.setPower(1);
        status = Status.IN;
    }

    public void out() {
        rollers.setPower(-1);
        status = Status.OUT;
    }

    public void off() {
        rollers.setPower(0);
        status = Status.OFF;
    }
}
