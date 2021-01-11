package org.firstinspires.ftc.teamcode.Quals;

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
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
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

public abstract class QualsSuperclass extends LinearOpMode {

    // ROBOT OBJECTS -------------------------------------------------------------------------------

    // Constants
    public Constants constants = new Constants();
    public int initCurrent = 0;
    public int initTotal;

    // Drivetrain
    public Drivetrain drivetrain = new Drivetrain();
    double vertical, horizontal, rotation;
    double max, kSlow;

    // Wobble Mech
    public WobbleMech wobbleMech = new WobbleMech();

    // Intake
    public Intake intake = new Intake();

    // Shooter
    public Shooter shooter = new Shooter();

    // Vision
    public Vision vision = new Vision();

    // Controller
    public Deadline gamepadRateLimit = new Deadline(Constants.GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

    // METHODS -------------------------------------------------------------------------------------

    // General Robot Methods =======================================================================

    public void initialize(boolean isAuto) {

        // Telemetry ===============================================================================
        telemetry.setAutoClear(false);
        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        sleep(500);

        if (isAuto)
            initTotal = 5;
        else
            initTotal = 4;

        // Drivetrain ==============================================================================
        drivetrain.imu = hardwareMap.get(BNO055IMU.class, "imu");
        drivetrain.frontLeft = (DcMotorEx)hardwareMap.dcMotor.get("frontLeft");
        drivetrain.frontRight = (DcMotorEx)hardwareMap.dcMotor.get("frontRight");
        drivetrain.backLeft = (DcMotorEx)hardwareMap.dcMotor.get("backLeft");
        drivetrain.backRight = (DcMotorEx)hardwareMap.dcMotor.get("backRight");
        drivetrain.initialize();
        initCurrent++;
        telemetry.addLine("Drivetrain initialized " + "(" + initCurrent + "/" + initTotal + ")");
        telemetry.update();
        sleep(500);

        // Shooter =================================================================================
        shooter.launcher = (DcMotorEx)hardwareMap.dcMotor.get("launcher");
        shooter.trigger = hardwareMap.servo.get("trigger");
        shooter.initialize();
        initCurrent++;
        telemetry.addLine("Shooter initialized " + "(" + initCurrent + "/" + initTotal + ")");
        telemetry.update();
        sleep(500);

        // Intake ==================================================================================
        intake.topRoller = (DcMotorEx)hardwareMap.dcMotor.get("topRoller");
        intake.bottomRoller = (DcMotorEx)hardwareMap.dcMotor.get("bottomRoller");
        intake.initialize();
        initCurrent++;
        telemetry.addLine("Intake initialized " + "(" + initCurrent + "/" + initTotal + ")");
        telemetry.update();
        sleep(500);

        // Wobble Mech =============================================================================
        wobbleMech.arm = (DcMotorEx)hardwareMap.dcMotor.get("arm");
        wobbleMech.lClaw = hardwareMap.servo.get("lClaw");
        wobbleMech.rClaw = hardwareMap.servo.get("rClaw");
        wobbleMech.initialize();
        initCurrent++;
        telemetry.addLine("Wobble Mech initialized " + "(" + initCurrent + "/" + initTotal + ")");
        telemetry.update();
        sleep(500);

        if (isAuto) {

            // Vision ==============================================================================
            vision.webcamName = hardwareMap.get(WebcamName.class, "Webcam");
            vision.initialize();
            initCurrent++;
            telemetry.addLine("Vision initialized " + "(" + initCurrent + "/" + initTotal + ")");
            telemetry.update();
            sleep(500);

            telemetry.addLine();
            telemetry.addLine("Load wobble goal and press 'A', or press 'B' to cancel...");
            telemetry.update();

            while (wobbleMech.initK == 0) {

                if (gamepad1.a) {
                    // Set wobble goal to pre-loaded position
                    wobbleMech.clawClose();
                    sleep(2000);
                    wobbleMech.setArmPosition(WobbleMech.ArmPosition.REST, 0.2);

                    telemetry.addLine("Wobble goal loaded");
                    telemetry.update();

                    wobbleMech.initK = 1;
                    sleep(500);
                }

                // Cancel wobble goal pre-load
                if (gamepad1.b) {
                    // Reset wobble mech
                    resetWobbleMech();

                    telemetry.addLine("Wobble goal not loaded");
                    telemetry.update();

                    wobbleMech.initK = 1;
                    sleep(500);
                }

                // Break out of loop if initialization is stopped to prevent forced restart
                if (isStopRequested()) {
                    break;
                }
            }

        } else {
            resetWobbleMech();
        }

        // Telemetry ===============================================================================
        telemetry.addLine("Initialization Finished " + "(" + initCurrent + "/" + initTotal + ")");
        telemetry.update();
        sleep(500);

        // Display robot rotation
        telemetry.setAutoClear(true);

        while(!isStarted())
            displayTeleOpTelemetry();
    }

    // Telemetry ===================================================================================

    public void displayTeleOpTelemetry() {
        // Telemetry
        telemetry.addLine("=== DRIVETRAIN ===");
        telemetry.addData("Heading (Deg)", drivetrain.getHeading(false));
        telemetry.addData("Drive Mode", drivetrain.driveMode);
        telemetry.addLine();

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Velocity (ticks/s)", shooter.getVelocity());
        telemetry.addData("Power", shooter.launcherPower);
        telemetry.addData("Rings Loaded", shooter.ringsLoaded);
        telemetry.addLine();

        telemetry.addLine("=== WOBBLE MECH ===");
        telemetry.addData("Arm Position", wobbleMech.getArmPosition());
        telemetry.addData("Claw Position", wobbleMech.getClawPosition());
        telemetry.addLine();

        telemetry.addLine("=== INTAKE ===");
        telemetry.addData("Rollers", intake.status);
        telemetry.addLine();

        telemetry.update();
    }

    // Drive Methods ===============================================================================

    public void drive() {

        // FIELD-CENTRIC DRIVE -----------------------------------------------------------------

        vertical = -gamepad1.left_stick_y;
        horizontal= gamepad1.left_stick_x * drivetrain.DRIVE_STRAFE_CORRECTION; // Correction to counteract imperfect strafing
        rotation = gamepad1.right_stick_x;

        // Joystick deadzones
        if (Math.abs(vertical) < 0.05)
            vertical = 0;
        if (Math.abs(horizontal) < 0.05)
             horizontal= 0;
        if (Math.abs(rotation) < 0.05)
            rotation = 0;

        if (drivetrain.driveMode == Drivetrain.DriveMode.FIELD_CENTRIC) {

            // Math
            if (drivetrain.getHeading(true) < 0) {       // If theta is measured clockwise from zero reference

                drivetrain.temp = vertical * Math.cos(drivetrain.getHeading(true)) + horizontal * Math.sin(-drivetrain.getHeading(true));
                horizontal= -vertical * Math.sin(-drivetrain.getHeading(true)) + horizontal * Math.cos(drivetrain.getHeading(true));
                vertical = drivetrain.temp;
            }

            if (drivetrain.getHeading(true) >= 0) {    // If theta is measured counterclockwise from zero reference

                drivetrain.temp = vertical * Math.cos(drivetrain.getHeading(true)) - horizontal * Math.sin(drivetrain.getHeading(true));
                horizontal= vertical * Math.sin(drivetrain.getHeading(true)) + horizontal * Math.cos(drivetrain.getHeading(true));
                vertical = drivetrain.temp;
            }
        }

        // Assign calculated values to the power variables
        drivetrain.flpower = vertical + horizontal + rotation;
        drivetrain.frpower = vertical - horizontal - rotation;
        drivetrain.blpower = vertical - horizontal + rotation;
        drivetrain.brpower = vertical + horizontal - rotation;

        // Find the greatest motor power
        max = Math.max(Math.max(Math.abs(drivetrain.flpower), Math.abs(drivetrain.frpower)),
                              Math.max(Math.abs(drivetrain.blpower), Math.abs(drivetrain.brpower)));
        // Scale motor powers with the greatest motor power
        drivetrain.flpower /= max;
        drivetrain.frpower /= max;
        drivetrain.blpower /= max;
        drivetrain.brpower /= max;

        /*
        // Motor powers are set to the power of 3 so that the drivetrain motors accelerates
        // exponentially instead of linearly
        // Note: you may consider, in the future, moving this code block to before the
        // max > 1 code block to see if that is better or worse performance, but I think
        // it will be worse because it may mess up proportions
        drivetrain.flpower = Math.pow(drivetrain.flpower, 3);
        drivetrain.blpower = Math.pow(drivetrain.blpower, 3);
        drivetrain.frpower = Math.pow(drivetrain.frpower, 3);
        drivetrain.brpower = Math.pow(drivetrain.brpower, 3);
         */

        // Motor power is decreased proportional to the horizontal trigger value to allow for more
        // precise robot control.
        kSlow = -2.0/3.0 * gamepad1.right_trigger + 1;
        drivetrain.flpower *= kSlow;
        drivetrain.frpower *= kSlow;
        drivetrain.blpower *= kSlow;
        drivetrain.brpower *= kSlow;

        drivetrain.setDrivePower(drivetrain.flpower, drivetrain.frpower, drivetrain.blpower, drivetrain.brpower);
    }

    public void driveTest() {

        // FIELD-CENTRIC DRIVE -----------------------------------------------------------------

        vertical = -gamepad1.left_stick_y;
         horizontal= gamepad1.left_stick_x * drivetrain.DRIVE_STRAFE_CORRECTION; // Correction to counteract imperfect strafing
        rotation = gamepad1.right_stick_x;

        // Joystick deadzones
        if (Math.abs(vertical) < 0.05)
            vertical = 0;
        if (Math.abs(horizontal) < 0.05)
             horizontal= 0;
        if (Math.abs(rotation) < 0.05)
            rotation = 0;

        if (drivetrain.driveMode == Drivetrain.DriveMode.FIELD_CENTRIC) {
            // Math
            if (drivetrain.getHeading(true) < 0) {       // If theta is measured clockwise from zero reference

                drivetrain.temp = vertical * Math.cos(drivetrain.getHeading(true)) + horizontal * Math.sin(-drivetrain.getHeading(true));
                horizontal= -vertical * Math.sin(-drivetrain.getHeading(true)) + horizontal * Math.cos(drivetrain.getHeading(true));
                vertical = drivetrain.temp;
            }

            if (drivetrain.getHeading(true) >= 0) {    // If theta is measured counterclockwise from zero reference

                drivetrain.temp = vertical * Math.cos(drivetrain.getHeading(true)) - horizontal * Math.sin(drivetrain.getHeading(true));
                horizontal= vertical * Math.sin(drivetrain.getHeading(true)) + horizontal * Math.cos(drivetrain.getHeading(true));
                vertical = drivetrain.temp;
            }
        }

        // Assign calculated values to the power variables
        drivetrain.flpower = vertical + horizontal + rotation;
        drivetrain.frpower = vertical - horizontal - rotation;
        drivetrain.blpower = vertical - horizontal + rotation;
        drivetrain.brpower = vertical + horizontal - rotation;

        // Find the greatest motor power
        max = Math.max(Math.max(Math.abs(drivetrain.flpower), Math.abs(drivetrain.frpower)),
                Math.max(Math.abs(drivetrain.blpower), Math.abs(drivetrain.brpower)));
        // Scale motor powers with the greatest motor power
        drivetrain.flpower /= max;
        drivetrain.frpower /= max;
        drivetrain.blpower /= max;
        drivetrain.brpower /= max;

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

    public void forward(double power, double inches) {

        double target = inches * drivetrain.DRIVE_TICKS_PER_INCH;

        if (opModeIsActive()) {

            drivetrain.resetDriveEncoders();
            drivetrain.setDriveTarget(target,
                    1, 1,
                    1, 1);
            drivetrain.setDriveRunMode();

            while (opModeIsActive() && drivetrain.driveIsBusy()) {
                drivetrain.setDrivePower(power);
            }

            drivetrain.setDrivePower(0);
            drivetrain.resetDriveMode();
        }
    }

    public void backward(double power, double inches) {

        double target = inches * drivetrain.DRIVE_TICKS_PER_INCH;

        if (opModeIsActive()) {

            drivetrain.resetDriveEncoders();
            drivetrain.setDriveTarget(target,
                    -1, -1,
                    -1, -1);
            drivetrain.setDriveRunMode();

            while (opModeIsActive() && drivetrain.driveIsBusy()) {
                drivetrain.setDrivePower(power);
            }

            drivetrain.setDrivePower(0);
            drivetrain.resetDriveMode();
        }
    }

    public void strafeRight(double power, double inches) {

        double target = inches * drivetrain.DRIVE_TICKS_PER_INCH;
        target *= drivetrain.DRIVE_STRAFE_CORRECTION;

        if (opModeIsActive()) {

            drivetrain.resetDriveEncoders();
            drivetrain.setDriveTarget(target,
                    1, -1,
                    -1, 1);
            drivetrain.setDriveRunMode();

            while (opModeIsActive() && drivetrain.driveIsBusy()) {
                drivetrain.setDrivePower(power);
            }

            drivetrain.setDrivePower(0);
            drivetrain.resetDriveMode();
        }
    }

    public void strafeLeft(double power, double inches) {

        double target = inches * drivetrain.DRIVE_TICKS_PER_INCH;
        target *= drivetrain.DRIVE_STRAFE_CORRECTION;

        if (opModeIsActive()) {

            drivetrain.resetDriveEncoders();
            drivetrain.setDriveTarget(target,
                    -1, 1,
                    1, -1);
            drivetrain.setDriveRunMode();

            while (opModeIsActive() && drivetrain.driveIsBusy()) {
                drivetrain.setDrivePower(power);
            }

            drivetrain.setDrivePower(0);
            drivetrain.resetDriveMode();
        }
    }

    public void rotateRight(double power, double deltaAngle) {

        double target = deltaAngle * drivetrain.DRIVE_TICKS_PER_DEGREE;

        if (opModeIsActive()) {

            drivetrain.resetDriveEncoders();
            drivetrain.setDriveTarget(target,
                    1, -1,
                    1, -1);
            drivetrain.setDriveRunMode();

            while (opModeIsActive() && drivetrain.driveIsBusy()) {
                drivetrain.setDrivePower(power);
            }

            drivetrain.setDrivePower(0);
            drivetrain.resetDriveMode();
        }
    }

    public void rotateLeft(double power, double deltaAngle) {

        double target = deltaAngle * drivetrain.DRIVE_TICKS_PER_DEGREE;

        if (opModeIsActive()) {

            drivetrain.resetDriveEncoders();
            drivetrain.setDriveTarget(target,
                    -1, 1,
                    -1, 1);
            drivetrain.setDriveRunMode();

            while (opModeIsActive() && drivetrain.driveIsBusy()) {
                drivetrain.setDrivePower(power);
            }

            drivetrain.setDrivePower(0);
            drivetrain.resetDriveMode();
        }
    }

    public void rotateToAngle(double power, double targetAngle) {

        double initialAngle = drivetrain.getHeading(false);
        double deltaAngle = targetAngle - initialAngle;
        if (Math.abs(deltaAngle) > 180) {
            deltaAngle = 360 - Math.abs(deltaAngle);
        }
        rotateRight(deltaAngle,power);
    }

    // Vision Methods ==============================================================================

    public void scanBitmap(boolean showPixelData) {

        int heightMid = (int)(vision.cropHeight/2.0);
        int[] yPos = {heightMid-9,heightMid-6,heightMid-3,heightMid,heightMid+3,heightMid+6,heightMid+9}; // 7 pixels top to bottom
        int[] xPos = {vision.cropWidth-1,0}; // Ring 1 (Rightmost pixel), Ring 4 (Leftmost pixel)
        int pixel, r, g, b, total;

        for (int i = 0; i < xPos.length; i++) {

            for (int j = 0; j < yPos.length; j++) {

                pixel = vision.croppedBitmap.getPixel(xPos[i],yPos[j]);
                r = Color.red(pixel);
                g = Color.green(pixel);
                b = Color.blue(pixel);
                total = r + g + b;

                // Print per pixel RGB to telemetry
                if (showPixelData) {
                    telemetry.addData("Red", r);
                    telemetry.addData("Green", g);
                    telemetry.addData("Blue", b);
                    telemetry.addData("Total", total);
                    telemetry.update();
                }

                // If lighting is too bright, increment check if r > g > b with noticeable difference
                // Else if under normal lighting, check if b < 17.5% of r+g
                // Else fail check (too dark)
                if (total > 375) {
                    if (r > g+40 && g > b && b > 70) {
                        telemetry.addLine("(" + i + "," + j + "): " + "*Bright Check Successful*");
                        telemetry.addLine();
                        telemetry.update();
                        vision.check++;
                    } else {
                        telemetry.addLine("(" + i + "," + j + "): " + "~Bright Check Failed~");
                        telemetry.addLine();
                        telemetry.update();
                    }
                } else if (total > 50) {
                    if ((17.5/100.0)*(r+g) > b) {
                        telemetry.addLine("(" + i + "," + j + "): " + "*Normal Check Successful*");
                        telemetry.addLine();
                        telemetry.update();
                        vision.check++;
                    } else {
                        telemetry.addLine("(" + i + "," + j + "): " + "~Normal Check Failed~");
                        telemetry.addLine();
                        telemetry.update();
                    }
                } else {
                    telemetry.addLine("(" + i + "," + j + "): " + "~Too Dark, Check Failed~");
                    telemetry.addLine();
                    telemetry.update();
                }
            }

            telemetry.addLine();
            telemetry.addData("Ring " + (i+1) + " Check Count",vision.check + "/7");
            telemetry.addLine();
            telemetry.update();

            if (vision.check >= 5) {
                vision.ringsDetected++;
            }

            vision.check = 0;
        }
    }

    public void vuforiaScanStack(boolean saveBitmaps, boolean showPixelData) {

        telemetry.setAutoClear(false);

        // Capture frame from camera
        vision.captureFrame();
        telemetry.addLine("Frame captured");
        telemetry.addLine();
        telemetry.update();

        if (vision.rgbImage != null) {

            // Transpose frame into bitmaps
            vision.setBitmaps();
            telemetry.addLine("Frame converted to bitmaps");
            telemetry.addLine();
            telemetry.update();

            // Save bitmaps to .png files
            if (saveBitmaps) {
                vision.saveBitmap("Bitmap", vision.bitmap);
                vision.saveBitmap("CroppedBitmap", vision.croppedBitmap);
                telemetry.addLine("Bitmaps saved to device");
                telemetry.addLine();
                telemetry.update();
            }

            // Scan bitmap for starter stack height
            scanBitmap(showPixelData);
            telemetry.addLine("Bitmap scan finished");
            telemetry.addData("Stack Height", vision.getStackHeight());
            telemetry.update();
        }
    }

    public void vuforiaScanTrackable() {

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        vision.targetsUltimateGoal.activate();

        // Change condition to something else later
        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            vision.targetVisible = false;
            for (VuforiaTrackable trackable : vision.allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    vision.targetVisible = true;
                    break;
                }
            }
            telemetry.update();
        }

        // Disable Tracking when we are done;
        vision.targetsUltimateGoal.deactivate();
    }

    // Wobble Mech Methods =========================================================================

    public void aim() {
        wobbleMech.clawOpen();
        sleep(750);
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.LOW, 0.3);
    }

    public void collect() {
        wobbleMech.clawClose();
        sleep(750);
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.REST, 0.2);
    }

    public void release() {
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.LOW, 0.2);
        sleep(750);
        wobbleMech.clawOpen();
        sleep(750);
        resetWobbleMech();
    }

    public void drop() {
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.HIGH, 0.2);
        sleep(750);
        wobbleMech.clawOpen();
        sleep(750);
        resetWobbleMech();
    }

    public void resetWobbleMech() {
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.REST, 0.3);
        wobbleMech.clawClose();
    }
}