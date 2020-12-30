package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.Util.Constants.motorTicksPerRev;

// Added left and right claws
// Updated initialize and claw methods for two servos
// Changed "motor" to "arm"
// Added armPower constant
// Updated arm angle positions to include vertical initialize position, removed reverse arm direction; moved tick conversion to method
// Switched claw open and close constants
// Added arm collect and release methods

public class WobbleMech {

    // Objects
    public DcMotorEx arm;
    public Servo lClaw, rClaw;

    // Constants
    private final double WOBBLE_TICKS_PER_REV = motorTicksPerRev[2];
    private final double WOBBLE_TICKS_PER_DEGREE = WOBBLE_TICKS_PER_REV/360;

    // Arm Angle Positions
    private final double[] wobbleAngles =
            {-45, 90, 15, 0}; // temp

    // Claw Positions
    private final double open = 0.0; // temp
    private final double close = 0.5325; // temp

    // Initialization Constants
    public double initK = 0;

    public WobbleMech() { }

    public void initialize() {

        // Arm
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setArmPosition(3, 0.4);
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Claw
        this.lClaw.setDirection(Servo.Direction.REVERSE);
        this.clawOpen();
    }

    public void setArmPosition(int position, double power) {
        this.arm.setTargetPosition((int)Math.round(wobbleAngles[position] * WOBBLE_TICKS_PER_DEGREE));
        this.setArmPower(power);
    }

    public int getArmPosition() {
        return this.arm.getCurrentPosition();
    }

    public void setArmPower(double power) {
        this.arm.setPower(power);
    }

    public void clawOpen() {
        this.lClaw.setPosition(open);
        this.rClaw.setPosition(open);
    }

    public void clawClose() {
        this.lClaw.setPosition(close);
        this.rClaw.setPosition(close);
    }

    public double getClawPosition() {
        return this.lClaw.getPosition();
    }
}
