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

public abstract class Shooter extends Subsystem {

    // Devices
    private static DcMotorEx launcherOne, launcherTwo;
    private static final String HM_L1 = "launcherOne";
    private static final String HM_L2 = "launcherTwo";
    private static Servo trigger;
    private static final String HM_TRIGGER = "trigger";

    // Trigger Constants
    private static final double TRIGGER_MIN = 0;
    private static final double TRIGGER_MAX = 0.12;
    private static TriggerPosition triggerPosition;

    // Launcher Constants
    public static final int ZERO_VELOCITY = 0;
    public static final int POWER_SHOT_VELOCITY = 1100; // tested
    public static final int HIGH_GOAL_VELOCITY = 1580; // tested
    public static final int[] VELOCITIES = {ZERO_VELOCITY,POWER_SHOT_VELOCITY,HIGH_GOAL_VELOCITY};
    private static int target, targetVelocity, velocityTolerance;
    private static ElapsedTime shotTimer;
    private static ElapsedTime rampUpTimer;

    // PID
    private static PIDCoefficients MOTOR_VELO_PID;
    private static double kV;
    private static double kA;
    private static double kStatic;
    private static ElapsedTime veloTimer;
    private static VelocityPIDFController veloController;
    private static double lastTargetVelocity;

    /**
     * Configures the hardware map, sets the trigger to the retracted position, and sets the defualt
     * target setting and target velocity to high goal.
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
        shotTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        rampUpTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        velocityTolerance = 20;

        // Trigger
        trigger.scaleRange(TRIGGER_MIN, TRIGGER_MAX);
        retractTrigger();

        // PID
        MOTOR_VELO_PID = new PIDCoefficients(0.00235, 0, 0.0000011);
        kV = 0.000465;
        kA = 0.00015;
        kStatic = 0;
        lastTargetVelocity = 0.0;
        veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
        for (LynxModule module : hm.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        veloTimer = new ElapsedTime();
        setTarget(VELOCITIES.length-1); // Set target to high goal

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
     * Updates the target setting and target velocity to the desired setting
     * @param nTarget Desired target setting
     * @see #VELOCITIES
     */
    public static void setTarget(int nTarget) {
        if (nTarget >= 0 && nTarget < VELOCITIES.length) {
            target = nTarget;
            targetVelocity = VELOCITIES[target];
        }
    }

    /**
     * Increments the target setting and updates the target velocity accordingly
     * @see #VELOCITIES
     */
    public static void increaseTarget() {
        if (target < VELOCITIES.length-1) target++;
        targetVelocity = VELOCITIES[target];
    }

    /**
     * Increments the target setting and updates the target velocity accordingly
     * @see #VELOCITIES
     */
    public static void decreaseTarget() {
        if (target > 0) target--;
        targetVelocity = VELOCITIES[target];
    }

    /**
     * @return Average velocity of the launcher motors
     */
    public static double getVelocity() {
        return (launcherOne.getVelocity()+launcherTwo.getVelocity())/2.0;
    }

    /**
     * @param motorName Name of the desired motor
     * @return Current velocity of the desired motor
     */
    public static double getVelocity(String motorName) {
        DcMotorEx motor = (DcMotorEx) hm.get(motorName);
        return motor.getVelocity();
    }

    /**
     * Starts the velocity timer. To be called right after {@link LinearOpMode#waitForStart()}.
     */
    public static void resetTimer() {
        veloTimer.reset();
    }

    /**
     * Runs the launcher at the target setting or the target velocity depending on the active
     * velocity control mode. Will only run if {@link #resetTimer()} has been called after start
     * outside of loop.
     */
    public static void refreshLauncher() {
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
            refreshLauncher();
        }
    }

    public static void setVelocityTolerance(int tolerance) {
        velocityTolerance = tolerance;
    }

    public static int getVelocityTolerance() {
        return velocityTolerance;

    }

    /**
     * Activates the launcher and pushes 3 rings; stops after the rings have been pushed.
     * @param target Desired target setting to run the launcher at
     * @see #VELOCITIES
     */
    public static void launchAll(int target) {
        double currentVelocity;
        int numShots = 0, check = 0;
        setTarget(target);
        veloTimer.reset();
        rampUpTimer.reset();
        shotTimer.reset();
        do {
            refreshLauncher();
            currentVelocity = getVelocity("launcherOne");
            double error = Math.abs(targetVelocity - currentVelocity);

            if (error <= velocityTolerance && check < 3)
                check++;
            else if (shotTimer.time() > 100 && rampUpTimer.time() > 350) {
                shootSingle();
                numShots++;
                shotTimer.reset();
            }
        } while (numShots < 3);

        sleep(250);
        Shooter.stop();
    }

    /**
     * Sets launcher motor powers and target setting to zero and updates the target velocity.
     */
    public static void stop() {
        launcherOne.setPower(0);
        launcherTwo.setPower(0);
        target = 0;
        targetVelocity = VELOCITIES[target];
    }

    /**
     * Appends Shooter data to telemetry. For the launcher, the data is displayed for launcherOne
     * @param expanded Shows expanded data for troubleshooting.
     */
    public static void appendTelemetry(boolean expanded) {
        tm.addLine("=== SHOOTER ===");
        tm.addData("Avg. Velocity", getVelocity());

        if (expanded) {
            tm.addLine("\nVelocities");
            tm.addData("L1 Velocity", getVelocity("launcherOne"));
            tm.addData("L2 Velocity", getVelocity("launcherTwo"));

            tm.addLine("\nTargets");
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
