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
    public DcMotorEx topRoller, bottomRoller;

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
        topRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        off();
    }

    public void in() {
        topRoller.setPower(1);
        bottomRoller.setPower(1);
        status = Status.IN;
    }

    public void out() {
        topRoller.setPower(-1);
        bottomRoller.setPower(-1);
        status = Status.OUT;
    }

    public void off() {
        topRoller.setPower(0);
        bottomRoller.setPower(0);
        status = Status.OFF;
    }
}
