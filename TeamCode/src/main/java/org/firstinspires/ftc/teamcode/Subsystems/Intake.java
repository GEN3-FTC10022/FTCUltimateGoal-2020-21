package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.Util.Constants.motorTicksPerRev;
/**
*this class was made to controll the intake system on the robot
*/
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
    /**
    * Status was made to show the status of the robot 
    * IN is to tell if the motor is taking in 
    * OUT is to tell if the motor are pushing out 
    * OFF is to tell if the motor is doing nothing 
    */
    public enum Status {
        IN,
        OUT,
        OFF
    }
    /**
    * Position was made to  tell the posotion of the intake 
    * UP is to tell if the intake is Up
    * DOWN is to tell if the intake is DOWN
    */
    public enum Position {
        UP,
        DOWN;
    }
    /**
    * initialize is the called when it is start up 
    * first is move the intake up
    * the it sets up the motor with out encoders 
    * then it turns off 
    */
    public void initialize() {

        // Release
        up();

        // Arm
        roller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        off();
    }
    /**
    * in turn the motor on to take in rings and sets status to IN
    */
    public void in() {
        roller.setPower(power);
        status = Status.IN;
    }
    /**
    * out turn the motor on to put out rings and sets status to OUT
    */
    public void out() {
        roller.setPower(-power);
        status = Status.OUT;
    }
    /**
    * off turn the motor off and sets status to OFF
    */
    public void off() {
        roller.setPower(0);
        status = Status.OFF;
    }
    /**
    * up moves the intake up and sets position to UP
    */
    public void up() {
        release.setPosition(up);
        position = Position.UP;
    }
    /**
    * down moves the intake down and sets position to DOWN
    */
    public void down() {
        release.setPosition(down);
        position = Position.DOWN;
    }
}
