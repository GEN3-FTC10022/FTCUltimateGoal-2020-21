package org.firstinspires.ftc.teamcode.Experimental;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static android.graphics.Bitmap.createBitmap;
import static android.graphics.Bitmap.createScaledBitmap;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.Util.Constants.motorTicksPerRev;

/*
    Updated for Drive Testing
 */

public abstract class TestingSuperclass extends LinearOpMode {

    // ROBOT OBJECTS -------------------------------------------------------------------------------

    public int initCurrent = 0;
    public final int initTotal = 4;

    // Constants
    public Constants constants = new Constants();

    // Drivetrain
    public Drivetrain drivetrain = new Drivetrain();

    // METHODS -------------------------------------------------------------------------------------

    // Robot Initialization
    public void initialize() {

        // Telemetry
        telemetry.setAutoClear(false);
        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        sleep(500);

        // Device Initialization

        telemetry.addLine("Vision initialized" + "(" + initCurrent + "/" + initTotal + ")");
        telemetry.update();
        sleep(500);

        // Drivetrain
        drivetrain.imu = hardwareMap.get(BNO055IMU.class, "imu");

        drivetrain.initialize();
        initCurrent++;

        telemetry.addLine("Drivetrain initialized" + "(" + initCurrent + "/" + initTotal + ")");
        telemetry.update();
        sleep(500);

        telemetry.addLine("Shooter initialized" + "(" + initCurrent + "/" + initTotal + ")");
        telemetry.update();
        sleep(500);

        telemetry.addLine("Wobble Mech initialized" + "(" + initCurrent + "/" + initTotal + ")");
        telemetry.update();
        sleep(500);

        // Telemetry
        telemetry.addLine("Initialization Finished" + "(" + initCurrent + "/" + initTotal + ")");
        telemetry.update();
        sleep(500);

        // Display robot rotation
        telemetry.setAutoClear(true);

        while(!isStarted()) {
            telemetry.addData("FC Heading (Deg)", Math.toDegrees(drivetrain.getHeading(true)));
            telemetry.addData("Heading (Deg)", drivetrain.getHeading(false));
            telemetry.update();
        }
    }

    // Drivetrain
    public void drive(boolean isFieldCentric) {

        // FIELD-CENTRIC DRIVE -----------------------------------------------------------------

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double clockwise = gamepad1.right_stick_x;

        // Joystick deadzones
        if (Math.abs(forward) < 0.05)
            forward = 0;
        if (Math.abs(right) < 0.05)
            right = 0;
        if (Math.abs(clockwise) < 0.05)
            clockwise = 0;

        if (isFieldCentric) {

            // Math
            if (drivetrain.getHeading(true) < 0) {       // If theta is measured clockwise from zero reference

                drivetrain.temp = forward * Math.cos(drivetrain.getHeading(true)) + right * Math.sin(-drivetrain.getHeading(true));
                right = -forward * Math.sin(-drivetrain.getHeading(true)) + right * Math.cos(drivetrain.getHeading(true));
                forward = drivetrain.temp;
            }

            if (drivetrain.getHeading(true) >= 0) {    // If theta is measured counterclockwise from zero reference

                drivetrain.temp = forward * Math.cos(drivetrain.getHeading(true)) - right * Math.sin(drivetrain.getHeading(true));
                right = forward * Math.sin(drivetrain.getHeading(true)) + right * Math.cos(drivetrain.getHeading(true));
                forward = drivetrain.temp;
            }
        }

        // Assign calculated values to the power variables
        drivetrain.flpower = forward + right + clockwise;
        drivetrain.frpower = forward - right - clockwise;
        drivetrain.blpower = forward - right + clockwise;
        drivetrain.brpower = forward + right - clockwise;

        // if you have the testing time, maybe remove this one day and see if it causes any
        // problems?
        // Find the maximum of the powers
        double max = Math.max(  Math.max(Math.abs(drivetrain.flpower), Math.abs(drivetrain.frpower)),
                Math.max(Math.abs(drivetrain.blpower), Math.abs(drivetrain.brpower))  );
        // Use this to make sure no motor powers are above 1 (the max value the motor can accept)
        if (max > 1) {

            drivetrain.flpower /= max;
            drivetrain.frpower /= max;
            drivetrain.blpower /= max;
            drivetrain.brpower /= max;
        }

        // Motor powers are set to the power of 3 so that the drivetrain motors accelerates
        // exponentially instead of linearly
        // Note: you may consider, in the future, moving this code block to before the
        // max > 1 code block to see if that is better or worse performance, but I think
        // it will be worse because it may mess up proportions
        drivetrain.flpower = Math.pow(drivetrain.flpower, 3);
        drivetrain.blpower = Math.pow(drivetrain.blpower, 3);
        drivetrain.frpower = Math.pow(drivetrain.frpower, 3);
        drivetrain.brpower = Math.pow(drivetrain.brpower, 3);

        // Motor Power is decreased while the right trigger is held down to allow for more
        // precise robot control
        if (gamepad1.right_trigger > 0.8) {

            drivetrain.flpower /= 3;
            drivetrain.frpower /= 3;
            drivetrain.blpower /= 3;
            drivetrain.brpower /= 3;
        }

        // If the trigger is held down, but not pressed all the way down, motor power will
        // slow down proportionally to how much the trigger is pressed
        else if (gamepad1.right_trigger > 0.1) {

            double driveSlow = -0.8 * gamepad1.right_trigger + 1;

            drivetrain.flpower *= driveSlow;
            drivetrain.frpower *= driveSlow;
            drivetrain.blpower *= driveSlow;
            drivetrain.brpower *= driveSlow;
        }

        drivetrain.setPowerAll(drivetrain.flpower, drivetrain.frpower, drivetrain.blpower, drivetrain.brpower);
    }

    public void driveTest(boolean isFieldCentric) {

        // FIELD-CENTRIC DRIVE -----------------------------------------------------------------

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double clockwise = gamepad1.right_stick_x;

        // Joystick deadzones
        if (Math.abs(forward) < 0.05)
            forward = 0;
        if (Math.abs(right) < 0.05)
            right = 0;
        if (Math.abs(clockwise) < 0.05)
            clockwise = 0;

        if (isFieldCentric) {

            // Math
            if (drivetrain.getHeading(true) < 0) {       // If theta is measured clockwise from zero reference

                drivetrain.temp = forward * Math.cos(drivetrain.getHeading(true)) + right * Math.sin(-drivetrain.getHeading(true));
                right = -forward * Math.sin(-drivetrain.getHeading(true)) + right * Math.cos(drivetrain.getHeading(true));
                forward = drivetrain.temp;
            }

            if (drivetrain.getHeading(true) >= 0) {    // If theta is measured counterclockwise from zero reference

                drivetrain.temp = forward * Math.cos(drivetrain.getHeading(true)) - right * Math.sin(drivetrain.getHeading(true));
                right = forward * Math.sin(drivetrain.getHeading(true)) + right * Math.cos(drivetrain.getHeading(true));
                forward = drivetrain.temp;
            }
        }

        // Assign calculated values to the power variables
        drivetrain.flpower = forward + right + clockwise;
        drivetrain.frpower = forward - right - clockwise;
        drivetrain.blpower = forward - right + clockwise;
        drivetrain.brpower = forward + right - clockwise;

        // if you have the testing time, maybe remove this one day and see if it causes any
        // problems?
        // Find the maximum of the powers
        double max = Math.max(  Math.max(Math.abs(drivetrain.flpower), Math.abs(drivetrain.frpower)),
                Math.max(Math.abs(drivetrain.blpower), Math.abs(drivetrain.brpower))  );
        // Use this to make sure no motor powers are above 1 (the max value the motor can accept)
        if (max > 1) {

            drivetrain.flpower /= max;
            drivetrain.frpower /= max;
            drivetrain.blpower /= max;
            drivetrain.brpower /= max;
        }

        // Motor powers are set to the power of 3 so that the drivetrain motors accelerates
        // exponentially instead of linearly
        // Note: you may consider, in the future, moving this code block to before the
        // max > 1 code block to see if that is better or worse performance, but I think
        // it will be worse because it may mess up proportions
        drivetrain.flpower = Math.pow(drivetrain.flpower, 3);
        drivetrain.blpower = Math.pow(drivetrain.blpower, 3);
        drivetrain.frpower = Math.pow(drivetrain.frpower, 3);
        drivetrain.brpower = Math.pow(drivetrain.brpower, 3);

        // Motor Power is decreased while the right trigger is held down to allow for more
        // precise robot control
        if (gamepad1.right_trigger > 0.8) {

            drivetrain.flpower /= 3;
            drivetrain.frpower /= 3;
            drivetrain.blpower /= 3;
            drivetrain.brpower /= 3;
        }

        // If the trigger is held down, but not pressed all the way down, motor power will
        // slow down proportionally to how much the trigger is pressed
        else if (gamepad1.right_trigger > 0.1) {

            double driveSlow = -0.8 * gamepad1.right_trigger + 1;

            drivetrain.flpower *= driveSlow;
            drivetrain.frpower *= driveSlow;
            drivetrain.blpower *= driveSlow;
            drivetrain.brpower *= driveSlow;
        }

        // Round powers for testing
        if (drivetrain.flpower >= 0.5) {
            drivetrain.flpower = 1;
        } else if (drivetrain.flpower <= -0.5) {
            drivetrain.flpower = -1;
        } else {
            drivetrain.flpower = 0;
        }

        if (drivetrain.frpower >= 0.5) {
            drivetrain.frpower = 1;
        } else if (drivetrain.frpower <= -0.5) {
            drivetrain.frpower = -1;
        } else {
            drivetrain.frpower = 0;
        }

        if (drivetrain.blpower >= 0.5) {
            drivetrain.blpower = 1;
        } else if (drivetrain.blpower <= -0.5) {
            drivetrain.blpower = -1;
        } else {
            drivetrain.blpower = 0;
        }

        if (drivetrain.brpower >= 0.5) {
            drivetrain.brpower = 1;
        } else if (drivetrain.brpower <= -0.5) {
            drivetrain.brpower = -1;
        } else {
            drivetrain.brpower = 0;
        }
    }
}
