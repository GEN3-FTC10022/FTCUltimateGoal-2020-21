package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.Subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public abstract class Intake extends Subsystem {

    // Objects
    public static DcMotorEx rollers;
    public static Servo release;

    // Constants
    private static Direction direction;
    private static Position position;
    private static final double POWER = 1;

    public Intake() { }

    /**
     * Roller Directions - IN, OUT, OFF
     */
    public enum Direction {
        IN,
        OUT,
        OFF
    }

    /**
     * @return Current direction of the intake rollers
     * @see Intake.Direction
     */
    public static Direction getDirection() {
        return direction;
    }

    /**
     * Locks the intake and turns off the rollers
     */
    public static void initialize(String hmRollers, String hmRelease) {

        // Hardware Map
        rollers = hm.get(DcMotorEx.class, hmRollers);
        release = hm.get(Servo.class, hmRelease);

        // Rollers
        rollers.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Release
        release.scaleRange(0, 0.3);

        lock();
        off();

        tm.addLine("Intake initialized");
        tm.update();
    }

    /**
     * Runs the rollers in the default direction
     */
    public static void in() {
        rollers.setPower(POWER);
        direction = Direction.IN;
    }

    /**
     * Runs the rollers in the reverse direction
     */
    public static void out() {
        rollers.setPower(-POWER);
        direction = Direction.OUT;
    }

    /**
     * Turns the rollers off
     */
    public static void off() {
        rollers.setPower(0);
        direction = Direction.OFF;
    }

    /**
     * Intake Positions - LOCKED, DROPPED
     */
    public enum Position {
        LOCKED,
        DROPPED;
    }

    /**
     * @return Current position of the intake system
     * @see Intake.Position
     */
    public static Position getPosition() {
        return position;
    }

    /**
     * Locks the intake
     */
    public static void lock() {
        release.setPosition(1);
        position = Position.LOCKED;
    }

    /**
     * Drops the intake
     */
    public static void drop() {
        release.setPosition(0);
        position = Position.DROPPED;
    }

    public static void appendTelemetry(boolean expanded) {
        tm.addLine("=== INTAKE ===");
        tm.addData("Direction", getDirection());
        tm.addData("Position", getPosition());

        if (expanded) {
            tm.addLine("\n:: Rollers ::");
            tm.addData("Motor Type", rollers.getMotorType());
            tm.addData("Controller", rollers.getController());
            tm.addData("Port Number", rollers.getPortNumber());
            tm.addData("Current", rollers.getCurrent(CurrentUnit.AMPS));
            tm.addData("Current Alert", rollers.getCurrentAlert(CurrentUnit.AMPS));
            tm.addData("Over Current", rollers.isOverCurrent());

            tm.addLine("\n:: Release ::");
            tm.addData("Min Position", release.MIN_POSITION);
            tm.addData("Max Position", release.MAX_POSITION);
            tm.addData("Current Position", release.getPosition());
            tm.addData("Controller", release.getController());
            tm.addData("Port Number", release.getPortNumber());
        }
    }
}
