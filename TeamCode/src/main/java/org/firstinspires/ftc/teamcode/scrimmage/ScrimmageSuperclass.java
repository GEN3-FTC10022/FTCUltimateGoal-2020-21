package org.firstinspires.ftc.teamcode.scrimmage;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class ScrimmageSuperclass extends LinearOpMode {

    // ROBOT OBJECTS -------------------------------------------------------------------------------

    // Drivetrain
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // REV Sensors
    public BNO055IMU imu;
    public Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);;
    public float angle = orientation.thirdAngle; // temp

    // CONTROL CONSTANTS ---------------------------------------------------------------------------

    public final double WHEEL_DIAMETER_INCHES = 4;
    public final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    public final double DRIVE_TICKS_PER_MOTOR_REV = 0; // temp
    public final double DRIVE_GEAR_REDUCTION = 0; // temp
    public final double DRIVE_TICKS_PER_INCH = (((DRIVE_TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_INCHES));
    public final double DRIVE_TICKS_PER_DEGREE = 0; // temp

    // METHODS -------------------------------------------------------------------------------------

    // Robot Initialization
    public void initialize() {

        // Device Initialization

        // Telemetry
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        // Drivetrain
        frontLeft = (DcMotorEx)hardwareMap.dcMotor.get("frontLeft");
        frontRight = (DcMotorEx)hardwareMap.dcMotor.get("frontRight");
        backLeft = (DcMotorEx)hardwareMap.dcMotor.get("backLeft");
        backRight = (DcMotorEx)hardwareMap.dcMotor.get("backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // REV Sensors
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        // Telemetry
        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    // Drive Methods
    public void forward(double pow, double inches) {

        double target = inches * DRIVE_TICKS_PER_INCH;

        if (opModeIsActive()) {

            resetDriveEncoders();
            setDriveTarget(target,
                    1, 1,
                    1, 1);
            setDriveMode();

            while (opModeIsActive() && driveIsBusy()) {
                setDrivePower(pow);
            }

            setDrivePower(0);
            resetDriveMode();
        }
    }

    public void backward(double pow, double inches) {

        double target = inches * DRIVE_TICKS_PER_INCH;

        if (opModeIsActive()) {

            resetDriveEncoders();
            setDriveTarget(target,
                    -1, -1,
                    -1, -1);
            setDriveMode();

            while (opModeIsActive() && driveIsBusy()) {
                setDrivePower(pow);
            }

            setDrivePower(0);
            resetDriveMode();
        }
    }

    public void strafeRight(double pow, double inches) {

        double target = inches * DRIVE_TICKS_PER_INCH;

        if (opModeIsActive()) {

            resetDriveEncoders();
            setDriveTarget(target,
                    1, -1,
                    -1, 1);
            setDriveMode();

            while (opModeIsActive() && driveIsBusy()) {
                setDrivePower(pow);
            }

            setDrivePower(0);
            resetDriveMode();
        }
    }

    public void strafeLeft(double pow, double inches) {

        double target = inches * DRIVE_TICKS_PER_INCH;

        if (opModeIsActive()) {

            resetDriveEncoders();
            setDriveTarget(target,
                    -1, 1,
                    1, -1);
            setDriveMode();

            while (opModeIsActive() && driveIsBusy()) {
                setDrivePower(pow);
            }

            setDrivePower(0);
            resetDriveMode();
        }
    }

    public void rotateRight(double pow, double angle) {

        double target = angle * DRIVE_TICKS_PER_DEGREE;

        if (opModeIsActive()) {

            resetDriveEncoders();
            setDriveTarget(target,
                    1, -1,
                    1, -1);
            setDriveMode();

            while (opModeIsActive() && driveIsBusy()) {
                setDrivePower(pow);
            }

            setDrivePower(0);
            resetDriveMode();
        }
    }

    public void rotateLeft(double pow, double angle) {

        double target = angle * DRIVE_TICKS_PER_DEGREE;

        if (opModeIsActive()) {

            resetDriveEncoders();
            setDriveTarget(target,
                    -1, 1,
                    -1, 1);
            setDriveMode();

            while (opModeIsActive() && driveIsBusy()) {
                setDrivePower(pow);
            }

            setDrivePower(0);
            resetDriveMode();
        }
    }

    // UTILITY METHODS -----------------------------------------------------------------------------

    // Drivetrain

    public void resetDriveEncoders() {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setDriveTarget(double dist, double fl, double fr, double bl, double br) {

        frontLeft.setTargetPosition((int) (fl * dist));
        backLeft.setTargetPosition((int) (bl * dist));
        frontRight.setTargetPosition((int) (fr * dist));
        backRight.setTargetPosition((int) (br * dist));
    }

    public void setDriveMode() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean driveIsBusy() {

        if (frontLeft.isBusy() && backLeft.isBusy() && frontRight.isBusy() && backRight.isBusy())
            return true;
        else
            return false;
    }

    public void setDrivePower(double pow) {

        frontLeft.setPower(pow);
        backLeft.setPower(pow);
        frontRight.setPower(pow);
        backRight.setPower(pow);
    }

    public void resetDriveMode() {

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // General

    public double getHeading() {

        double robotHeading;

        // Convert -180 to 180 into 0-360
        if (angle < 0)
            robotHeading = angle * -1;
        else if (angle > 0)
            robotHeading = 360 - angle;
        else
            robotHeading = 0;

        return robotHeading;
    }
}
