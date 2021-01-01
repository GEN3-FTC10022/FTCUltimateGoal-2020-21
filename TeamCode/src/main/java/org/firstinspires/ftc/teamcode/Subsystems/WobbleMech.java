package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    public double armPosition;

    // Claw Positions
    private final double open = 0.0; // temp
    private final double close = 0.5325; // temp
    public String clawPosition;

    // Initialization Constants
    public double initK = 0;

    public WobbleMech() { }

    public void initialize() {

        // Arm
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setArmPosition(3, 0.4);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Claw
        lClaw.setDirection(Servo.Direction.REVERSE);
        clawOpen();
    }

    public void setArmPosition(int position, double power) {
        arm.setTargetPosition((int)Math.round(wobbleAngles[position] * WOBBLE_TICKS_PER_DEGREE));
        setArmPower(power);
        armPosition = position;
    }

    public int getArmPosition() {
        return arm.getCurrentPosition();
    }

    public void setArmPower(double power) {
        arm.setPower(power);
    }

    public void clawOpen() {
        lClaw.setPosition(open);
        rClaw.setPosition(open);
        clawPosition = "Open";
    }

    public void clawClose() {
        lClaw.setPosition(close);
        rClaw.setPosition(close);
        clawPosition = "Close";
    }
}
