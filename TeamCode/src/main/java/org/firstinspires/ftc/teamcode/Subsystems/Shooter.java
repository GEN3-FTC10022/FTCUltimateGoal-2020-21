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
    private int targetVelocity;
    public int ringsLoaded;
    private final double LOW_SHOT_VELOCITY = 1300; // temp
    private final double MID_SHOT_VELOCITY = 1340; // temp
    private final double POWER_SHOT_VELOCITY = 1440; // temp
    private final double HIGH_SHOT_VELOCITY = 1460; // tested
    public final double[] VELOCITIES = {LOW_SHOT_VELOCITY,MID_SHOT_VELOCITY,POWER_SHOT_VELOCITY,HIGH_SHOT_VELOCITY};

    // PID
    public PIDFCoefficients launcherEncoderPIDF = new PIDFCoefficients(7.5,3,3.5,0);
    public PIDFCoefficients launcherPositionPIDF = new PIDFCoefficients(0,0,0,0);

    /**
     * Creates a shooter object
     */
    public Shooter() { }

    /**
     * Contains the position of the trigger as one of two options: push or retract
     */
    public enum TriggerPosition {
        PUSH,
        RETRACT;
    }

    /**
     * Initializes the shooter by retracting the trigger, reversing the motor, setting it's
     * zeroPowerBehavior to BRAKE, and putting it in run mode to travel at a targeted
     * velocity using PID.
     */
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

        setTargetVelocity(3);
        ringsLoaded = 3;
    }

    /**
     * Puts the trigger in the push position and updates the triggerPosition variable
     */
    public void pushTrigger() {
        trigger.setPosition(push);
        triggerPosition = TriggerPosition.PUSH;
    }

    /**
     * Puts the trigger in the retract position and updates the triggerPosition variable
     */
    public void retractTrigger() {
        trigger.setPosition(retract);
        triggerPosition = TriggerPosition.RETRACT;
    }

    /**
     * @return the trigger position as either push or retract
     */
    public TriggerPosition getTriggerPosition() {
        return triggerPosition;
    }

    /**
     * @return the current velocity of the second launcher
     */
    public double getVelocity() {
        return launcherTwo.getVelocity();
    }

    public void setTargetVelocity(int setting) {
        targetVelocity = setting;
    }

    /**
     * @return the current target velocity
     */
    public double getTargetVelocity() {
        return VELOCITIES[targetVelocity];
    }

    /**
     * Increases the target velocity by a decided upon value and updates the
     * target velocity for the launcher
     */
    public void increaseVelocity() {
        if (targetVelocity < VELOCITIES.length-1)
            targetVelocity++;
        runShooter();
    }

    /**
     * Decreases the target velocity by a decided upon value and updates the
     * target velocity for the launcher
     */
    public void decreaseVelocity() {
        if (targetVelocity > 0)
            targetVelocity--;
        runShooter();
    }

    /**
     * Sets the velocity of the launcher to the specified target speed
     */
    public void runShooter() {
        // launcherOne.setVelocity(targetVelocity);
        launcherTwo.setVelocity(VELOCITIES[targetVelocity]);
    }
}
