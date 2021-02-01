package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.Util.Constants.motorTicksPerRev;

public class WobbleMech {

    // Objects
    public DcMotorEx arm;
    public Servo lClaw, rClaw;

    // Constants
    private final double WOBBLE_TICKS_PER_REV = motorTicksPerRev[2];
    private final double WOBBLE_TICKS_PER_DEGREE = WOBBLE_TICKS_PER_REV/360;
    public int initK;

    // Arm Angle Positions
    private final double[] wobbleAngles =
            {0, 60, 135}; // temp
    private ArmPosition armPosition;

    // Claw Positions
    private final double open = 0.0; // temp
    private final double close = 0.5225; // temp
    private ClawPosition clawPosition;

    public WobbleMech() { }

    public enum ArmPosition {
        REST,
        HIGH,
        LOW;
    }

    public enum ClawPosition {
        OPEN,
        CLOSE;
    }

    /** Sets the arm to the rest position and opens the claw.
     * @see org.firstinspires.ftc.teamcode.Quals.QualsSuperclass
     */
    public void initialize() {

        // Arm
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setArmPosition(ArmPosition.REST, 0.2);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Claw
        rClaw.setDirection(Servo.Direction.REVERSE);
        clawOpen();

        // Initialization Constant
        initK = 0;
    }

    /** Sets the arm to the speicified position at the given power.
     * @param armPosition Positions of the Wobble Mech (REST/HIGH/LOW)
     * @param power Power set to the motor
     */
    public void setArmPosition(ArmPosition armPosition, double power) {
        int index;
        switch (armPosition) {
            case HIGH:
                index = 1;
                break;
            case LOW:
                index = 2;
                break;
            default: // REST
                index = 0;
        }
        arm.setTargetPosition((int)Math.round(wobbleAngles[index] * WOBBLE_TICKS_PER_DEGREE));
        setArmPower(power);
        this.armPosition = armPosition;
    }


    /**
     * Outputs the current position of the arm.
     *
     * @return armPosition The position of the arm (REST/HIGH/LOW).
     */
    public ArmPosition getArmPosition() {
        return armPosition;
    }

    public void setArmPower(double power) {
        arm.setPower(power);
    }

    /**
     * Opens the claws
     */
    public void clawOpen() {
        lClaw.setPosition(open);
        rClaw.setPosition(open);
        clawPosition = ClawPosition.OPEN;
    }

    public void clawClose() {
        lClaw.setPosition(close);
        rClaw.setPosition(close);
        clawPosition = ClawPosition.CLOSE;
    }

    public ClawPosition getClawPosition() {
        return clawPosition;
    }
}
