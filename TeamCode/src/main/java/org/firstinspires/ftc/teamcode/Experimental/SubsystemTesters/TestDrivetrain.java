package org.firstinspires.ftc.teamcode.Experimental.SubsystemTesters;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Experimental.TestingSuperclass;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Util.Constants;

@TeleOp(name = "Subsystems: Drivetrain Test")
public class TestDrivetrain extends LinearOpMode {

    private Constants constants = new Constants();
    private Drivetrain drivetrain = new Drivetrain();
    private double vertical, horizontal, rotation, max, kSlow;

    @Override
    public void runOpMode() {

        initialize();

        telemetry.setAutoClear(true);

        waitForStart();

        doTeleOp();

        // doAuto();
    }

    private void initialize() {

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

        telemetry.setAutoClear(true);
        while(!isStarted())
            displayTeleOpTelemetry();
    }

    public void doTeleOp() {
        while (opModeIsActive()) {

            // TELEMETRY ===========================================================================

            displayTeleOpTelemetry();

            // Drive
            drive();

            // Switch Modes
            if (gamepad1.back && constants.back == 0)
                constants.back++;
            else if (!gamepad1.back && constants.back == 1) {
                if (drivetrain.driveMode == Drivetrain.DriveMode.FIELD_CENTRIC) {
                    drivetrain.setMode(Drivetrain.DriveMode.ROBOT_CENTRIC);
                } else {
                    drivetrain.setMode(Drivetrain.DriveMode.FIELD_CENTRIC);
                }
                constants.back--;
            }
        }
    }

    public void doAuto() {
        rotateRight(0.6, 225);
        sleep(125);
        rotateToAngle(0.6, 0, true);
        sleep(10000);
    }

    public void displayTeleOpTelemetry() {

        telemetry.addLine("=== DRIVETRAIN ===");
        telemetry.addData("Heading (Deg)", drivetrain.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Heading Correction (Deg)", drivetrain.getHeadingCorrection(AngleUnit.DEGREES));
        telemetry.addData("Drive Mode", drivetrain.driveMode);
        telemetry.update();
    }

    // Drive Methods ===============================================================================

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

            if (Math.abs(targetAngle - drivetrain.getHeading(AngleUnit.DEGREES)) > 0.3) {
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
}