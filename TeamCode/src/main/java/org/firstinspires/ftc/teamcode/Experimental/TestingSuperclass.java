package org.firstinspires.ftc.teamcode.Experimental;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;

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

    // Drivetrain
    public Drivetrain drivetrain = new Drivetrain();
    public double vertical, horizontal, rotation, max, kSlow;

    // Vision
    public Vision vision = new Vision();


    // METHODS -------------------------------------------------------------------------------------

    // Robot Initialization
    public void initialize(boolean isAuto) {

        telemetry.setAutoClear(false);

        drivetrain.imu = hardwareMap.get(BNO055IMU.class, "imu");
        drivetrain.frontLeft = (DcMotorEx)hardwareMap.dcMotor.get("frontLeft");
        drivetrain.frontRight = (DcMotorEx)hardwareMap.dcMotor.get("frontRight");
        drivetrain.backLeft = (DcMotorEx)hardwareMap.dcMotor.get("backLeft");
        drivetrain.backRight = (DcMotorEx)hardwareMap.dcMotor.get("backRight");
        drivetrain.initialize();

        telemetry.addLine("Setting Correction...");
        telemetry.update();
        sleep(500);

        while (drivetrain.getHeading(AngleUnit.DEGREES) != 0) {
            drivetrain.setHeadingCorrectionDegrees();
            telemetry.addData("Correction", drivetrain.getHeadingCorrection(AngleUnit.DEGREES));
            telemetry.update();
            sleep(200);

            // Break out of loop if initialization is stopped to prevent forced restart
            if (isStopRequested()) {
                break;
            }
        }

        telemetry.addLine("Drivetrain initialized");
        telemetry.update();
        sleep(500);

        // Shooter =================================================================================
        shooter.launcherOne = (DcMotorEx)hardwareMap.dcMotor.get("launcherOne");
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

        if (isAuto) {

            // Vision ==============================================================================
            vision.webcamName = hardwareMap.get(WebcamName.class, "Webcam");
            vision.initialize();
            telemetry.addLine("Vision initialized");
            telemetry.update();
            sleep(500);

            telemetry.addLine();
            telemetry.addLine("Load wobble goal and press 'A', or press 'B' to cancel...");
            telemetry.update();

            while (wobbleMech.initK == 0) {

                if (gamepad1.a) {
                    // Set wobble goal to pre-loaded position
                    wobbleMech.clawClose();
                    sleep(1000);
                    wobbleMech.setArmPosition(WobbleMech.ArmPosition.REST);

                    telemetry.addLine("Wobble goal loaded");
                    telemetry.update();

                    wobbleMech.initK++;
                    sleep(500);
                }

                // Cancel wobble goal pre-load
                if (gamepad1.b) {
                    // Reset wobble mech
                    resetWobbleMech();

                    telemetry.addLine("Wobble goal not loaded");
                    telemetry.update();

                    wobbleMech.initK++;
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
    }

    public void displayTeleOpTelemetry() {

        telemetry.addLine("=== DRIVETRAIN ===");
        telemetry.addData("Heading (Deg)", drivetrain.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Heading Correction (Deg)", drivetrain.getHeadingCorrection(AngleUnit.DEGREES));
        telemetry.addData("Drive Mode", drivetrain.driveMode);
        telemetry.addLine();

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Velocity (ticks/s)", shooter.getVelocity());
        telemetry.addData("Target Velocity (ticks/s)", shooter.getTargetVelocity());
        telemetry.addData("PID Encoder", shooter.launcherTwo.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("PID Position", shooter.launcherTwo.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addLine();

        telemetry.addLine("=== WOBBLE MECH ===");
        telemetry.addData("Arm Position", wobbleMech.getArmPosition());
        telemetry.addData("Arm RunMOde", wobbleMech.arm.getMode());
        telemetry.addData("Claw Position", wobbleMech.getClawPosition());
        telemetry.addLine();

        telemetry.addLine("=== INTAKE ===");
        telemetry.addData("Rollers", intake.status);
        telemetry.addData("Position", intake.position);
        telemetry.addLine();

        telemetry.update();
    }

    public void drive() {

        // FIELD-CENTRIC DRIVE =================================================================

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
            if (drivetrain.getHeading(AngleUnit.RADIANS) < 0) {
                // If theta is measured clockwise from zero reference

                drivetrain.temp = vertical * Math.cos(drivetrain.getHeading(AngleUnit.RADIANS))
                        + horizontal * Math.sin(-drivetrain.getHeading(AngleUnit.RADIANS));
                horizontal = - vertical * Math.sin(-drivetrain.getHeading(AngleUnit.RADIANS))
                        + horizontal * Math.cos(drivetrain.getHeading(AngleUnit.RADIANS));
                vertical = drivetrain.temp;
            }

            if (drivetrain.getHeading(AngleUnit.RADIANS) >= 0) {
                // If theta is measured counterclockwise from zero reference

                drivetrain.temp = vertical * Math.cos(drivetrain.getHeading(AngleUnit.RADIANS))
                        - horizontal * Math.sin(drivetrain.getHeading(AngleUnit.RADIANS));
                horizontal = vertical * Math.sin(drivetrain.getHeading(AngleUnit.RADIANS))
                        + horizontal * Math.cos(drivetrain.getHeading(AngleUnit.RADIANS));
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

        // Motor power is decreased proportional to the horizontal trigger value to allow for more
        // precise robot control.
        kSlow = -2.0/3.0 * gamepad1.right_trigger + 1;
        drivetrain.flpower *= kSlow;
        drivetrain.frpower *= kSlow;
        drivetrain.blpower *= kSlow;
        drivetrain.brpower *= kSlow;

        drivetrain.setDrivePower(drivetrain.flpower, drivetrain.frpower, drivetrain.blpower, drivetrain.brpower);
    }

    public void driveReading() {

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
            if (drivetrain.getHeading(AngleUnit.RADIANS) < 0) {       // If theta is measured clockwise from zero reference

                drivetrain.temp = vertical * Math.cos(drivetrain.getHeading(AngleUnit.RADIANS)) + horizontal * Math.sin(-drivetrain.getHeading(AngleUnit.RADIANS));
                horizontal= -vertical * Math.sin(-drivetrain.getHeading(AngleUnit.RADIANS)) + horizontal * Math.cos(drivetrain.getHeading(AngleUnit.RADIANS));
                vertical = drivetrain.temp;
            }

            if (drivetrain.getHeading(AngleUnit.RADIANS) >= 0) {    // If theta is measured counterclockwise from zero reference

                drivetrain.temp = vertical * Math.cos(drivetrain.getHeading(AngleUnit.RADIANS)) - horizontal * Math.sin(drivetrain.getHeading(AngleUnit.RADIANS));
                horizontal= vertical * Math.sin(drivetrain.getHeading(AngleUnit.RADIANS)) + horizontal * Math.cos(drivetrain.getHeading(AngleUnit.RADIANS));
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

    public void rotateToAngle(double power, double targetAngle, boolean displayInfo) {

        telemetry.setAutoClear(false);
        double initialAngle = drivetrain.getHeading(AngleUnit.DEGREES);
        double deltaAngle = targetAngle - initialAngle;
        if (Math.abs(deltaAngle) > 180) {
            deltaAngle = 360 - Math.abs(deltaAngle);
        }

        if (displayInfo) {
            telemetry.addData("Initial Angle", initialAngle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Delta Angle", deltaAngle);
            telemetry.update();
        }

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

            if (Math.abs(targetAngle - drivetrain.getHeading(AngleUnit.DEGREES)) > 0.5) {
                rotateToAngle(power, targetAngle, false);
            }

            drivetrain.setDrivePower(0);
            drivetrain.resetDriveMode();
        }

        if (displayInfo) {
            telemetry.addData("Final Angle", drivetrain.getHeading(AngleUnit.DEGREES));
            telemetry.addLine("Rotate Finished");
            telemetry.update();
        }
    }

    // Wobble Mech Methods =========================================================================

    public void aim() {
        wobbleMech.clawOpen();
        sleep(500);
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.LOW);
    }

    public void collect() {
        wobbleMech.clawClose();
        sleep(500);
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.REST);
    }

    public void place() {
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.LOW);
        sleep(500);
        wobbleMech.clawOpen();
        sleep(500);
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.REST);
    }

    public void drop() {
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.HIGH);
        sleep(500);
        wobbleMech.clawOpen();
        sleep(500);
        resetWobbleMech();
    }

    public void resetWobbleMech() {
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.REST);
        sleep(500);
        wobbleMech.zeroArm();
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
            sleep(500);
            displayTeleOpTelemetry();
        }
    }

    // Vision Methods ==============================================================================

    private void scanBitmap(boolean showPixelData) {

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
}
