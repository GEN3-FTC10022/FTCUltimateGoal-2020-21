package org.firstinspires.ftc.teamcode.Experimental;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import static android.graphics.Bitmap.createBitmap;
import static android.graphics.Bitmap.createScaledBitmap;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.Util.Constants.motorTicksPerRev;

public abstract class TestingSuperclass extends LinearOpMode {

    // ROBOT OBJECTS -------------------------------------------------------------------------------

    // Constants
    public Constants constants = new Constants();

    // Shooter
    public Shooter shooter = new Shooter();

    // Intake
    public Intake intake = new Intake();

    // Wobble Mech
    public WobbleMech wobbleMech = new WobbleMech();


    // METHODS -------------------------------------------------------------------------------------

    // Robot Initialization
    public void initialize() {

        telemetry.setAutoClear(false);

        // Shooter =================================================================================
        // shooter.launcherOne = (DcMotorEx)hardwareMap.dcMotor.get("launcherOne");
        shooter.launcherTwo = (DcMotorEx)hardwareMap.dcMotor.get("launcherTwo");
        shooter.trigger = hardwareMap.servo.get("trigger");
        shooter.initialize();
        telemetry.addLine("Shooter initialized");
        telemetry.update();
        sleep(500);

        // Intake ==================================================================================
        intake.roller = (DcMotorEx)hardwareMap.dcMotor.get("rollers");
        intake.release = hardwareMap.servo.get("release");
        intake.initialize();
        telemetry.addLine("Intake initialized");
        telemetry.update();
        sleep(500);

        // Wobble Mech =============================================================================
        wobbleMech.arm = (DcMotorEx)hardwareMap.dcMotor.get("arm");
        wobbleMech.lClaw = hardwareMap.servo.get("lClaw");
        wobbleMech.rClaw = hardwareMap.servo.get("rClaw");
        wobbleMech.initialize();
        telemetry.addLine("Wobble Mech initialized");
        telemetry.update();
        sleep(500);

        // Display telemetry
        telemetry.setAutoClear(true);
        while(!isStarted())
            displayTeleOpTelemetry();
    }

    public void displayTeleOpTelemetry() {
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Velocity (ticks/s)", shooter.getVelocity());
        telemetry.addData("Target Velocity (ticks/s)", shooter.getTargetVelocity());
        telemetry.addData("Rings Loaded", shooter.ringsLoaded);
        telemetry.addData("PID Encoder", shooter.launcherTwo.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("PID Position", shooter.launcherTwo.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.update();
    }

    // Wobble Mech Methods =========================================================================

    public void aim() {
        wobbleMech.clawOpen();
        sleep(750);
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.LOW);
    }

    public void collect() {
        wobbleMech.clawClose();
        sleep(750);
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.REST);
    }

    public void place() {
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.LOW);
        sleep(750);
        resetWobbleMech();
    }

    public void drop() {
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.HIGH);
        sleep(750);
        wobbleMech.clawOpen();
        sleep(750);
        resetWobbleMech();
    }

    public void resetWobbleMech() {
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.REST);
        wobbleMech.clawClose();
    }

    // Shooter Methods =============================================================================

    public void shootSingle() {
        shooter.pushTrigger();
        sleep(100);
        shooter.retractTrigger();
        shooter.ringsLoaded--;
        if (shooter.ringsLoaded == 0)
            shooter.ringsLoaded = 3;
    }

    public void shootAll() {
        for (int i = 0; i < 3; i++) {
            shootSingle();
            sleep(250);
            displayTeleOpTelemetry();
        }
    }
}
