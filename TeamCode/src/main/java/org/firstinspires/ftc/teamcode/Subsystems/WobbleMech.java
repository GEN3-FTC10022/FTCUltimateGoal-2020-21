package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.Subsystem;

import static org.firstinspires.ftc.teamcode.Util.Constants.NEVEREST_CLASSIC_60_TICKS_PER_REV;

public abstract class WobbleMech extends Subsystem {

    // Devices
    private static DcMotorEx arm;
    private static final String HM_ARM = "arm";
    private static Servo lClaw, rClaw;
    private static final String HM_LCLAW = "lClaw";
    private static final String HM_RCLAW = "rClaw";

    // Constants
    private static final double WOBBLE_TICKS_PER_REV = NEVEREST_CLASSIC_60_TICKS_PER_REV;
    private static final double WOBBLE_GEAR_REDUCTION = 2;
    private static final double WOBBLE_TICKS_PER_DEGREE = (WOBBLE_TICKS_PER_REV * WOBBLE_GEAR_REDUCTION)/360.0;
    private static final double ARM_POWER = 0.8;
    private static final double[] WOBBLE_ANGLES = {0, 50, 125};
    private static ArmPosition armPosition;

    private static final double CLAW_MIN = 0;
    private static final double CLAW_MAX = 0.5225;
    private static ClawPosition clawPosition;

    private static ControlMode controlMode;

    /**
     * Configures the hardware map, sets the control mode to assisted, sets the arm to the rest
     * position, and opens the claw.
     */
    public static void initialize() {

        // Hardware Map
        arm = hm.get(DcMotorEx.class, HM_ARM);
        lClaw = hm.get(Servo.class, HM_LCLAW);
        rClaw = hm.get(Servo.class, HM_RCLAW);

        // Arm
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Claw
        rClaw.setDirection(Servo.Direction.REVERSE);
        rClaw.scaleRange(CLAW_MIN, CLAW_MAX);
        lClaw.scaleRange(CLAW_MIN, CLAW_MAX);

        controlMode = ControlMode.ASSISTED;
        setArmPosition(ArmPosition.REST);
        clawClose();

        tm.addLine("Wobble Mech initialized");
        tm.update();
    }

    /**
     * Control Modes - ASSISTED, MANUAL
     */
    public enum ControlMode {
        ASSISTED,
        MANUAL;
    }

    /**
     * @return Current system control mode
     * @see WobbleMech.ControlMode
     */
    public static ControlMode getControlMode() {
        return controlMode;
    }

    /**
     * Sets the active control mode to the argument
     * @param controlMode The desired system control mode.
     * @see WobbleMech.ControlMode
     */
    public static void setControlMode(ControlMode controlMode) {
        WobbleMech.controlMode = controlMode;
    }

    /**
     * Arm Positions - REST, HIGH, LOW
     */
    public enum ArmPosition {

        /**
         * This is the zero position of the wobble mech; the arm is rested against the mechanical
         * stop.
         */
        REST,

        /**
         * This is the position to drop the wobble goal over the field walls.
         */
        HIGH,

        /**
         * This is the position where the arm is parallel to the ground to collect the wobble goal.
         */
        LOW;
    }

    /**
     * @return Current position of the arm
     * @see WobbleMech.ArmPosition
     */
    public static ArmPosition getArmPosition() {
        return armPosition;
    }

    /**
     * Moves the arm to the specified position.
     * @param armPosition Target arm position
     * @see WobbleMech.ArmPosition
     */
    private static void setArmPosition(ArmPosition armPosition) {
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
        arm.setTargetPosition((int)Math.round(WOBBLE_ANGLES[index] * WOBBLE_TICKS_PER_DEGREE));
        arm.setPower(ARM_POWER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WobbleMech.armPosition = armPosition;
    }

    /**
     * @deprecated Currently causes zeroing error
     */
    @Deprecated
    private static void zeroArm() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Claw Positions - OPEN, CLOSED
     */
    public enum ClawPosition {
        OPEN,
        CLOSED;
    }

    /**
     * @return Current position of the claw
     * @see WobbleMech.ClawPosition
     */
    public static ClawPosition getClawPosition() {
        return clawPosition;
    }

    /**
     * Opens the claws
     */
    public static void clawOpen() {
        lClaw.setPosition(0);
        rClaw.setPosition(0);
        clawPosition = ClawPosition.OPEN;
    }

    /**
     * Closes the claws
     */
    public static void clawClose() {
        lClaw.setPosition(1);
        rClaw.setPosition(1);
        clawPosition = ClawPosition.CLOSED;
    }

    /**
     * Moves the arm to the low position and opens the claws.
     */
    public static void aim() {
        WobbleMech.clawOpen();
        sleep(500);
        WobbleMech.setArmPosition(WobbleMech.ArmPosition.LOW);
    }

    /**
     * Closes the claws resets the wobble mech.
     */
    public static void collect() {
        WobbleMech.clawClose();
        sleep(1000);
        reset();
    }

    /**
     * Moves the arm to the low position and opens the claws. Then, resets the wobble mech.
     */
    public static void place() {
        WobbleMech.setArmPosition(WobbleMech.ArmPosition.LOW);
        sleep(1000);
        WobbleMech.clawOpen();
        sleep(500);
        reset();
    }

    /**
     * Moves the arm to the high position and opens the claws. Then, resets the wobble mech.
     */
    public static void drop() {
        WobbleMech.setArmPosition(WobbleMech.ArmPosition.HIGH);
        sleep(500);
        WobbleMech.clawOpen();
        sleep(500);
        reset();
    }

    /**
     * Moves the arm to the rest position and closes the claws.
     */
    public static void reset() {
        WobbleMech.setArmPosition(WobbleMech.ArmPosition.REST);
        sleep(500);
        WobbleMech.clawClose();
    }

    /**
     * Applies positive power to the arm motor.
     */
    public static void armUp() {
        arm.setPower(-ARM_POWER);
    }

    /**
     * Applies negative power to the arm motor.
     */
    public static void armDown() {
        arm.setPower(ARM_POWER);
    }

    /**
     * Applies zero power to the arm motor.
     */
    public static void armStop() {
        arm.setPower(0);
    }

    /**
     * Set RunMode of the arm to the desired RunMode
     * @param runMode The desired RunMode
     */
    public static void setRunMode(DcMotor.RunMode runMode) {
        arm.setMode(runMode);
    }

    /**
     * Appends Wobble Mech data to telemetry. For the claw, the data is displayed for the left
     * servo unless otherwise specified.
     * @param expanded Shows expanded data for troubleshooting.
     */
    public static void appendTelemetry(boolean expanded) {
        tm.addLine("=== Wobble Mech ===");
        tm.addData("Arm Position", getArmPosition());
        tm.addData("Claw Position", getClawPosition());
        tm.addData("Control Mode", getControlMode());

        if (expanded) {
            tm.addLine("\n:: Arm ::");
            tm.addData("Motor Type", arm.getMotorType());
            tm.addData("Controller", arm.getController());
            tm.addData("Port Number", arm.getPortNumber());
            tm.addData("Current", arm.getCurrent(CurrentUnit.AMPS));
            tm.addData("Current Alert", arm.getCurrentAlert(CurrentUnit.AMPS));
            tm.addData("Over Current", arm.isOverCurrent());
            tm.addData("Run Mode", arm.getMode());
            tm.addData("Encoder PID", arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            tm.addData("Position PID", arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

            tm.addLine("\n:: Claw ::");
            tm.addData("Min Position", lClaw.MIN_POSITION);
            tm.addData("Max Position", lClaw.MAX_POSITION);
            tm.addData("Current Position", lClaw.getPosition());
            tm.addData("L.Claw Controller", lClaw.getController());
            tm.addData("R.Claw Controller", lClaw.getController());
            tm.addData("L.Claw Port Number", lClaw.getPortNumber());
            tm.addData("R.Claw Port Number", lClaw.getPortNumber());
        }

        tm.addLine();
    }
}
