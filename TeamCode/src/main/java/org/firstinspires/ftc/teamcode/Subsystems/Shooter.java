package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Util.Constants.YELLOWJACKET_5202_MAX_RPM;
import static org.firstinspires.ftc.teamcode.Util.Constants.motorTicksPerRev;

public class Shooter {

    // Shooter Objects
    public DcMotorEx mShooter;
    public Servo sTrigger;

    // Servo Constants
    public final double sRetract = 0.0; // temp
    public final double sPush = 0.3; // temp

    // Shooter Constants
    private final double SHOOTER_TICKS_PER_REV = motorTicksPerRev[3];
    private final double SHOOTER_MAX_REV_PER_MIN = 0.8 * YELLOWJACKET_5202_MAX_RPM; // Max Shooter Vel @ 80% of motor max
    public final double SHOOTER_MAX_TICKS_PER_SEC = SHOOTER_MAX_REV_PER_MIN * (SHOOTER_TICKS_PER_REV/60.0);

    public final double percentModifier = 0.05;
    public double percentVelocity;

    public Shooter() {

    }

    public void initialize() {

        // Trigger
        this.sTrigger.setDirection(Servo.Direction.REVERSE);
        retractTrigger();

        // Shooter
        this.mShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.mShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        percentVelocity = 0;
        this.runShooter();
    }

    public void pushTrigger() {
        this.sTrigger.setPosition(sPush);
    }

    public void retractTrigger() {
        this.sTrigger.setPosition(sRetract);
    }

    public double getTriggerPosition() {
        return this.sTrigger.getPosition();
    }

    public double getVelocity() {
        return this.mShooter.getVelocity();
    }

    public void increasePower() {
        if (percentVelocity < 1) {
            this.percentVelocity += percentModifier;
        }
        this.runShooter();
    }

    public void decreasePower() {
        if (percentVelocity > 0) {
            this.percentVelocity -= percentModifier;
        }
        this.runShooter();
    }

    public void setPercentVelocity(double percent) {
        this.percentVelocity = percent;
    }

    public void runShooter() {
        this.mShooter.setPower(percentVelocity);
        // this.mShooter.setVelocity(this.percentVelocity * this.SHOOTER_MAX_TICKS_PER_SEC);
    }
}
