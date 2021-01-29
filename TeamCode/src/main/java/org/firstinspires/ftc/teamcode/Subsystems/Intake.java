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
    public DcMotorEx roller;
    public Servo release;

    // Constants
    public Status status;
    public Position position;
    private final double power = 0.7;
    private final double up = 0;
    private final double down = 0.5;

    public Intake() { }

    public enum Status {
        IN,
        OUT,
        OFF
    }

    public enum Position {
        UP,
        DOWN;
    }

    public void initialize() {

        // Release
        up();

        // Arm
        roller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        off();
    }

    public void in() {
        roller.setPower(power);
        status = Status.IN;
    }

    public void out() {
        roller.setPower(-power);
        status = Status.OUT;
    }

    public void off() {
        roller.setPower(0);
        status = Status.OFF;
    }

    public void up() {
        release.setPosition(up);
        position = Position.UP;
    }

    public void down() {
        release.setPosition(down);
        position = Position.DOWN;
    }
}
