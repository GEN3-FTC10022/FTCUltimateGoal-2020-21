package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.ShooterTuning.VelocityPIDFController;
import org.firstinspires.ftc.teamcode.Util.Subsystem;

import static org.firstinspires.ftc.teamcode.Util.Constants.YELLOWJACKET_5202_MAX_RPM;
import static org.firstinspires.ftc.teamcode.Util.Constants.YELLOWJACKET_5202_TICKS_PER_REV;

public abstract class Shooter extends Subsystem {

    // Devices
    private static DcMotorEx launcherOne, launcherTwo;
    private static final String HM_L1 = "launcherOne";
    private static final String HM_L2 = "launcherTwo";
    private static Servo trigger;
    private static final String HM_TRIGGER = "trigger";

    // Constants
    private static final double TRIGGER_MIN = 0;
    private static final double TRIGGER_MAX = 0.12;
    private static TriggerPosition triggerPosition;

    private static final double LAUNCHER_TICKS_PER_REV = YELLOWJACKET_5202_TICKS_PER_REV;
    private static final double LAUNCHER_MAX_REV_PER_MIN = 0.9 * YELLOWJACKET_5202_MAX_RPM; // Max Shooter Vel @ 90% of motor max
    private static final double LAUNCHER_MAX_TICKS_PER_SECOND = LAUNCHER_MAX_REV_PER_MIN * (LAUNCHER_TICKS_PER_REV/60.0);

    public static final int ZERO_VELOCITY = 0;
    public static final int MID_GOAL_VELOCITY = 1000; // temp
    public static final int POWER_SHOT_VELOCITY = 1100; // tested
    public static final int HIGH_GOAL_VELOCITY = 1230; // tested
    private static final int[] VELOCITIES = {ZERO_VELOCITY,POWER_SHOT_VELOCITY,HIGH_GOAL_VELOCITY};
    private static int targetSetting;

    private static ControlMode controlMode;
    private static final double VELOCITY_MODIFIER = 20;
    private static int targetVelocity;

    // PID
    private static PIDCoefficients MOTOR_VELO_PID;
    private static double kV;
    private static double kA;
    private static double kStatic;
    private static ElapsedTime veloTimer;
    private static VelocityPIDFController veloController;
    private static double lastTargetVelocity;

    /**
     * Configures the hardware map, sets the VCM to preset, sets the trigger to the retracted
     * position, and sets the defualt target setting and manual target velocity to high goal.
     */
    public static void initialize() {

        // Hardware Map
        launcherOne = hm.get(DcMotorEx.class, HM_L1);
        launcherTwo = hm.get(DcMotorEx.class, HM_L2);
        trigger = hm.get(Servo.class, HM_TRIGGER);

        // Launcher
        launcherOne.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Trigger
        trigger.scaleRange(TRIGGER_MIN, TRIGGER_MAX);

        // PID
        MOTOR_VELO_PID = new PIDCoefficients(0.00235, 0, 0.0000011);
        kV = 0.000465;
        kA = 0.00015;
        kStatic = 0;
        veloTimer = new ElapsedTime();
        lastTargetVelocity = 0.0;
        veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
        for (LynxModule module : hm.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        controlMode = ControlMode.PRESET;
        retractTrigger();
        targetSetting = VELOCITIES.length-1; // Set target setting to high goal
        targetVelocity = HIGH_GOAL_VELOCITY; // Set target velocity to high goal

        tm.addLine("Shooter initialized");
        tm.update();
    }

    /**
     * Trigger Postions - EXTENDED, RETRACTED
     */
    public enum TriggerPosition {
        EXTENDED,
        RETRACTED
    }

    /**
     * @return Current position of the trigger
     */
    public static TriggerPosition getTriggerPosition() {
        return triggerPosition;
    }

    /**
     * Puts the trigger in the push position and updates the trigger position
     * @see Shooter.TriggerPosition
     */
    public static void pushTrigger() {
        trigger.setPosition(1);
        triggerPosition = TriggerPosition.EXTENDED;
    }

    /**
     * Puts the trigger in the retract position and updates the trigger position
     * @see Shooter.TriggerPosition
     */
    public static void retractTrigger() {
        trigger.setPosition(0);
        triggerPosition = TriggerPosition.RETRACTED;
    }

    /**
     * Velocity Control Mode - PRESET, MANUAL
     */
    public enum ControlMode {
        /**
         * Adjusting the launcher velocity in this mode will cycle through the 5 preset velocities.
         * @see #VELOCITIES
         */
        PRESET,

        /**
         * Adjusting the launcher velocity in this mode will apply a direct modifier to the target
         * velocity.
         * @see #VELOCITY_MODIFIER
         */
        MANUAL
    }

    /**
     * @return Current velocity control mode of the shooter
     */
    public static ControlMode getControlMode() {
        return controlMode;
    }

    /**
     * Sets the active velocity control mode to the argument
     * @param controlMode The desired velocity control mode.
     * @see ControlMode
     */
    public static void setControlMode(ControlMode controlMode) {
        Shooter.controlMode = controlMode;
    }

    /**
     * Sets the target velocity or target setting of the launcher depending on the active velocity
     * control mode.
     * @param k The desired target velocity in ticks per second or target setting.
     */
    public static void setTarget(int k) {
        if (controlMode == ControlMode.PRESET && k >= 0 && k < VELOCITIES.length)
            targetSetting = k;
        else if (controlMode == ControlMode.MANUAL && k <= LAUNCHER_MAX_TICKS_PER_SECOND)
            targetVelocity = k;
    }

    /**
     * @return Current target velocity in ticks per second or the current target setting depending
     * on the active velocity control mode. Returns -1 if the velocity control mode has not been
     * initialized.
     */
    public static double getTarget() {
        if (controlMode == ControlMode.PRESET)
            return targetSetting;
        else if (controlMode == ControlMode.MANUAL)
            return targetVelocity;
        else
            return -1;
    }

    /**
     * Increments the target setting or the target velocity depending on the active velocity control
     * mode.
     */
    public static void increaseVelocity() {
        if (controlMode == ControlMode.PRESET && targetSetting < VELOCITIES.length-1)
            targetSetting++;
        else if (controlMode == ControlMode.MANUAL && targetVelocity <= LAUNCHER_MAX_TICKS_PER_SECOND-VELOCITY_MODIFIER)
            targetVelocity += VELOCITY_MODIFIER;
    }

    /**
     * Decrements the target setting or the target velocity depending on the active velocity control
     * mode.
     */
    public static void decreaseVelocity() {
        if (controlMode == ControlMode.PRESET && targetSetting > 0)
            targetSetting--;
        else if (controlMode == ControlMode.MANUAL && targetVelocity >= -LAUNCHER_MAX_TICKS_PER_SECOND+VELOCITY_MODIFIER)
            targetVelocity -= VELOCITY_MODIFIER;
    }

    /**
     * Starts the velocity timer. To be called right after {@link LinearOpMode#waitForStart()}.
     */
    public static void startLauncher() {
        veloTimer.reset();
    }

    /**
     * Runs the launcher at the target setting or the target velocity depending on the active
     * velocity control mode. Will only run if {@link #startLauncher()} has been called after start
     * outside of loop.
     */
    public static void runLauncher() {

        if (controlMode == ControlMode.PRESET)
            veloController.setTargetVelocity(VELOCITIES[targetSetting]);
        else
            veloController.setTargetVelocity(targetVelocity);

        veloController.setTargetAcceleration((targetVelocity - lastTargetVelocity) / veloTimer.seconds());
        veloTimer.reset();
        lastTargetVelocity = targetVelocity;

        double power = veloController.update(launcherOne.getCurrentPosition(),launcherOne.getVelocity());
        launcherOne.setPower(power);
        launcherTwo.setPower(power);
    }

    /**
     * Launches one ring
     */
    public static void shootSingle() {
        pushTrigger();
        sleep(80);
        retractTrigger();
    }

    /**
     * Launches three rings
     */
    public static void shootAll() {
        for (int i = 0; i < 3; i++) {
            shootSingle();
            sleep(300);
        }
    }

    /**
     * Appends Shooter data to telemetry. For the launcher, the data is displayed for launcherOne
     * @param expanded Shows expanded data for troubleshooting.
     */
    public static void appendTelemetry(boolean expanded) {
        tm.addLine("=== SHOOTER ===");
        tm.addData("L1 Velocity", launcherOne.getVelocity());
        tm.addData("L2 Velocity", launcherTwo.getVelocity());
        if (controlMode == ControlMode.PRESET)
            tm.addData("Target", VELOCITIES[targetSetting]);
        else
            tm.addData("Target", targetVelocity);
        tm.addData("VCM", getControlMode());

        if (expanded) {
            tm.addLine("\nVelocities");
            tm.addData("Mid Goal", MID_GOAL_VELOCITY);
            tm.addData("Power Shot", POWER_SHOT_VELOCITY);
            tm.addData("High Goal", HIGH_GOAL_VELOCITY);

            tm.addLine("\n:: Trigger ::");
            tm.addData("Min Position", trigger.MIN_POSITION);
            tm.addData("Max Position", trigger.MAX_POSITION);
            tm.addData("Current Position", trigger.getPosition());
            tm.addData("Controller", trigger.getController());
            tm.addData("Port Number", trigger.getPortNumber());

            tm.addLine("\n:: Launcher ::");
            tm.addData("L1 Power", launcherOne.getPower());
            tm.addData("L2 Power", launcherTwo.getPower());
            tm.addData("Current", launcherOne.getCurrent(CurrentUnit.AMPS));
            tm.addData("Current Alert", launcherOne.getCurrentAlert(CurrentUnit.AMPS));
            tm.addData("Over Current", launcherOne.isOverCurrent());
            tm.addData("Run Mode", launcherOne.getMode());
            tm.addData("Encoder PID", launcherOne.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        }

        tm.addLine();
    }
}
