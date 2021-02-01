package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Util.Constants.YELLOWJACKET_5202_MAX_RPM;
import static org.firstinspires.ftc.teamcode.Util.Constants.motorTicksPerRev;

public class Shooter {

    // Shooter Objects
    public DcMotorEx launcherOne, launcherTwo;
    public Servo trigger;
    public TriggerPosition triggerPosition;

    // Servo Constants
    private final double retract = 0; // temp
    private final double push = 0.15; // temp

    // Shooter Constants
    private final double SHOOTER_TICKS_PER_REV = motorTicksPerRev[3];
    private final double SHOOTER_MAX_REV_PER_MIN = 0.8 * YELLOWJACKET_5202_MAX_RPM; // Max Shooter Vel @ 80% of motor max
    public final double SHOOTER_MAX_TICKS_PER_SECOND = SHOOTER_MAX_REV_PER_MIN * (SHOOTER_TICKS_PER_REV/60.0);

    public final double VELOCITY_MODIFIER = 20;
    private double targetVelocity;
    public int ringsLoaded;
    public final double LOW_SHOT_VELOCITY = 1300; // temp
    public final double MID_SHOT_VELOCITY = 1340; // temp
    public final double POWER_SHOT_VELOCITY = 1380; // temp
    public final double HIGH_SHOT_VELOCITY = 1660; // tested

    // PID
    public PIDFCoefficients launcherEncoderPIDF = new PIDFCoefficients(7.5,3,3.5,0);
    public PIDFCoefficients launcherPositionPIDF = new PIDFCoefficients(0,0,0,0);

    public Shooter() { }

    public enum TriggerPosition {
        PUSH,
        RETRACT;
    }

    public void initialize() {

        // Trigger
        retractTrigger();

        // Shooter
        // launcherOne.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        // launcherOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // launcherOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // launcherOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherEncoderPIDF);
        launcherTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherEncoderPIDF);
        // launcherOne.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, launcherPositionPIDF);
        launcherTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, launcherPositionPIDF);

        ringsLoaded = 3;
    }

    public void pushTrigger() {
        trigger.setPosition(push);
        triggerPosition = TriggerPosition.PUSH;
    }

    public void retractTrigger() {
        trigger.setPosition(retract);
        triggerPosition = TriggerPosition.RETRACT;
    }

    public TriggerPosition getTriggerPosition() {
        return triggerPosition;
    }

    public double getVelocity() {
        return launcherTwo.getVelocity();
    }

    public void increaseVelocity() {
        targetVelocity += VELOCITY_MODIFIER;
        runShooter();
    }

    public void decreaseVelocity() {
        targetVelocity -= VELOCITY_MODIFIER;
        runShooter();
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void runShooter() {
        // launcherOne.setVelocity(targetVelocity);
        launcherTwo.setVelocity(targetVelocity);
    }
}
