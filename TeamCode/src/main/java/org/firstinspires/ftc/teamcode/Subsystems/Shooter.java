package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Util.Constants.YELLOWJACKET_5202_MAX_RPM;
import static org.firstinspires.ftc.teamcode.Util.Constants.motorTicksPerRev;

public class Shooter {

    // Shooter Objects
    public DcMotorEx launcher;
    public Servo trigger;

    // Servo Constants
    public final double sRetract = 0.0; // temp
    public final double sPush = 0.3; // temp

    // Shooter Constants
    private final double SHOOTER_TICKS_PER_REV = motorTicksPerRev[3];
    private final double SHOOTER_MAX_REV_PER_MIN = 0.8 * YELLOWJACKET_5202_MAX_RPM; // Max Shooter Vel @ 80% of motor max
    public final double SHOOTER_MAX_TICKS_PER_SEC = SHOOTER_MAX_REV_PER_MIN * (SHOOTER_TICKS_PER_REV/60.0);

    public final double percentModifier = 0.05;
    public double percentVelocity;

    public Shooter() { }

    public void initialize() {

        // Trigger
        trigger.setDirection(Servo.Direction.REVERSE);
        retractTrigger();

        // Shooter
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        percentVelocity = 0;
        runShooter();
    }

    public void pushTrigger() {
        trigger.setPosition(sPush);
    }

    public void retractTrigger() {
        trigger.setPosition(sRetract);
    }

    public double getTriggerPosition() {
        return trigger.getPosition();
    }

    public double getVelocity() {
        return launcher.getVelocity();
    }

    public void increasePower() {
        if (percentVelocity < 1) {
            percentVelocity += percentModifier;
        }
        runShooter();
    }

    public void decreasePower() {
        if (percentVelocity > 0) {
            percentVelocity -= percentModifier;
        }
        runShooter();
    }

    public void setPercentVelocity(double percent) {
        percentVelocity = percent;
    }

    public void runShooter() {
        launcher.setPower(percentVelocity);
        // launcher.setVelocity(percentVelocity * SHOOTER_MAX_TICKS_PER_SEC);
    }
}
