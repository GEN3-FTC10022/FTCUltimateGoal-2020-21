package org.firstinspires.ftc.teamcode.Experimental.SubsystemTesters;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

            telemetry.setAutoClear(true);
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

            if (gamepad1.a && constants.a == 0)
                constants.a++;
            else if (!gamepad1.a && constants.a == 1) {
                rotateToAnglePID(0.8, -135);
                sleep(2000);
                rotateToAnglePID(0.8, 0);
                sleep(2000);
                constants.a--;
            }
        }
    }

    public void doAuto() {
        rotateToAnglePID(0.6, 90);
        sleep(30000);
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

    public void rotateToAnglePID(double power, double targetAngle) {
        telemetry.setAutoClear(false);
        double initialAngle = drivetrain.getHeading(AngleUnit.DEGREES);
        double deltaAngle = targetAngle - initialAngle;

        telemetry.addLine("\n Rotating...");
        telemetry.addData("Initial Angle", initialAngle);
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Delta Angle", deltaAngle);
        telemetry.update();

        rotate(power,deltaAngle);

        telemetry.addData("Rotate Finished", drivetrain.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(double power, double degrees) {
        // restart imu angle tracking.
        drivetrain.resetAngle();

        // If input degrees > 359, we cap at 359 with same sign as input.
        if (Math.abs(degrees) > 359) degrees = Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. We compute the p and I
        // values based on the input degrees and starting power level. We compute the tolerance %
        // to yield a tolerance value of about 1 degree.
        // Overshoot is dependant on the motor and gearing configuration, starting power, weight
        // of the robot and the on target tolerance.

        drivetrain.controller.reset();

        // Proportional factor can be found by dividing the max desired pid output by
        // the setpoint or target. Here 30% power is divided by 90 degrees (.30 / 90)
        // to get a P factor of .003. This works for the robot we testing this code with.
        // Your robot may vary but this way finding P works well in most situations.
        double p = Math.abs(power/degrees);

        // Integrative factor can be approximated by diving P by 100. Then you have to tune
        // this value until the robot turns, slows down and stops accurately and also does
        // not take too long to "home" in on the setpoint. Started with 100 but robot did not
        // slow and overshot the turn. Increasing I slowed the end of the turn and completed
        // the turn in a timely manner
        double i = p / 100.0;

        drivetrain.controller.setPID(p, i, 0);

        drivetrain.controller.setSetpoint(degrees);
        drivetrain.controller.setInputRange(0, degrees);
        drivetrain.controller.setOutputRange(0, power);
        drivetrain.controller.setTolerance(1.0 / Math.abs(degrees) * 100.0);
        telemetry.addData("Tolerance", (1.0 / Math.abs(degrees) * 100.0));
        telemetry.update();
        sleep(2000);
        drivetrain.controller.enable();

        // drivetrain.getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && drivetrain.getAngle() == 0)
            {
                drivetrain.setDrivePower(power,-power,power,-power);
                sleep(100);
            }

            do
            {
                power = drivetrain.controller.performPID(drivetrain.getAngle()); // power will be - on right turn.
                drivetrain.setDrivePower(-power,power,-power,power);
            } while (opModeIsActive() && !drivetrain.controller.onTarget());
        }
        else    // left turn.
            do
            {
                power = drivetrain.controller.performPID(drivetrain.getAngle()); // power will be + on left turn.
                drivetrain.setDrivePower(-power,power,-power,power);
            } while (opModeIsActive() && !drivetrain.controller.onTarget());

        // turn the motors off.
        drivetrain.setDrivePower(0);

        rotation = drivetrain.getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        drivetrain.resetAngle();
    }
}