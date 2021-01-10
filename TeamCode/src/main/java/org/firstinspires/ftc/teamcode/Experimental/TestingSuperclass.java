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

    // Controller
    public Deadline gamepadRateLimit = new Deadline(Constants.GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

    // METHODS -------------------------------------------------------------------------------------

    // Robot Initialization
    public void initialize() {

        // Shooter =================================================================================
        shooter.launcher = (DcMotorEx)hardwareMap.dcMotor.get("launcher");
        shooter.trigger = hardwareMap.servo.get("trigger");
        shooter.initialize();
        telemetry.addLine("Shooter initialized");
        telemetry.update();
        sleep(500);
    }

    public void displayTeleOpTelemetry() {

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Velocity (ticks/s)", shooter.getVelocity());
        telemetry.addData("Position", shooter.launcher.getCurrentPosition());
        // telemetry.addData("Target Velocity", shooter.getTargetVelocity());
        // telemetry.addData("Trigger", shooter.getTriggerPosition());
        // telemetry.addData("Percent Velocity", shooter.percentVelocity);
        // telemetry.addData("Max ticks/s", shooter.SHOOTER_MAX_TICKS_PER_SECOND);
        telemetry.addData("RunMode", shooter.launcher.getMode());
        telemetry.addData("Encoder PIDF", shooter.launcher.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("Position PIDF", shooter.launcher.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addLine();

        telemetry.update();
    }
}
