package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Util.Constants.YELLOWJACKET_5202_MAX_RPM;
import static org.firstinspires.ftc.teamcode.Util.Constants.motorTicksPerRev;

public class Shooter {

    // Shooter Objects
    public DcMotorEx mShooter;
    public Servo sTrigger;

    // Servo Constants
    private final double sRetract = 0.0; // temp
    private final double sPush = 0.45; // temp

    // Shooter Constants
    private final double SHOOTER_TICKS_PER_REV = motorTicksPerRev[3];
    private final double SHOOTER_MAX_REV_PER_MIN = 0.8 * YELLOWJACKET_5202_MAX_RPM; // Max Shooter Vel @ 80% of motor max
    private final double SHOOTER_MAX_TICKS_PER_SEC = SHOOTER_MAX_REV_PER_MIN * SHOOTER_TICKS_PER_REV/60.0;

    private final double percentModifier = 0.2;
    public double percentVelocity = 0;

    public Shooter() {

    }

    public void initialize() {
        // retractTrigger();
        this.mShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.mShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.mShooter.setVelocity(percentVelocity * SHOOTER_MAX_TICKS_PER_SEC);
    }

    public void pushTrigger() {
        this.sTrigger.setPosition(sPush);
    }

    public void retractTrigger() {
        this.sTrigger.setPosition(sRetract);
    }

    public void fireTrigger() {
        pushTrigger();
        retractTrigger();
    }

    public void increaseSpeed() {
        while (percentVelocity < 1) {
            this.percentVelocity += this.percentModifier;
        }
    }

    public void decreaseSpeed() {
        while (percentVelocity > 0) {
            this.percentVelocity -= this.percentModifier;
        }
    }

    public void setSpeed(double speed) {
        this.percentVelocity = speed;
    }

    public double getVelocity() {
        return this.mShooter.getVelocity();
    }

    public double getTriggerPosition() {
        return this.sTrigger.getPosition();
    }
}
