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
    public DcMotorEx launcher;
    public Servo trigger;
    public TriggerPosition triggerPosition;

    // Servo Constants
    private final double retract = 0.55; // temp
    private final double push = 0.2; // temp

    // Shooter Constants
    private final double SHOOTER_TICKS_PER_REV = motorTicksPerRev[3];
    private final double SHOOTER_MAX_REV_PER_MIN = 0.8 * YELLOWJACKET_5202_MAX_RPM; // Max Shooter Vel @ 80% of motor max
    public final double SHOOTER_MAX_TICKS_PER_SECOND = SHOOTER_MAX_REV_PER_MIN * (SHOOTER_TICKS_PER_REV/60.0);

    public final double percentModifier = 0.05;
    public double percentVelocity;

    // PID
    private PIDFCoefficients launcherPID = new PIDFCoefficients(100,0,0,0);
    private PIDFCoefficients launcherPIDZero = new PIDFCoefficients(0,0,0,0);

    public Shooter() { }

    public enum TriggerPosition {
        PUSH,
        RETRACT;
    }

    public void initialize() {

        // Trigger
        retractTrigger();

        // Shooter
        // launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        // launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // launcher.setVelocityPIDFCoefficients(0,0,0,0);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,launcherPID);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,launcherPIDZero);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setTargetPosition(-56);
        percentVelocity = 0;
        runShooter();
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
        return launcher.getVelocity();
    }

    public double getTargetVelocity() {
        return percentVelocity * SHOOTER_MAX_TICKS_PER_SECOND;
    }

    public void increasePower() {
        if (percentVelocity < 1) {
            percentVelocity += percentModifier;
        }
        runShooter();
    }

    public void decreasePower() {
        if (percentVelocity > 0) {
            percentVelocity += percentModifier;
        }
        runShooter();
    }

    public void setPercentVelocity(double percent) {
        percentVelocity = percent;
    }

    public void runShooter() {
        launcher.setVelocity(-percentVelocity * SHOOTER_MAX_TICKS_PER_SECOND);
    }
}
