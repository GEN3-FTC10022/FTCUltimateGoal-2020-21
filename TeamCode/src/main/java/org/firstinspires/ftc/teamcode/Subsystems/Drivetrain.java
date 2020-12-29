package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.Util.Constants.motorTicksPerRev;

public class Drivetrain {

    // motors
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // encoders
    public DcMotor leftEncoder, rightEncoder, horzEncoder;

    // REV IMU
    public BNO055IMU imu;
    public Orientation orientation;
    public float angle;

    // TeleOp variables
    public double temp;
    public double flpower, frpower, blpower, brpower;

    ElapsedTime runtime = new ElapsedTime();

    public int leftPos = 0, rightPos = 0, horzPos = 0;
    public double leftChange = 0, rightChange = 0, horzChange = 0;
    public double x = 0, y = 0;
    public double odoAngle = 0;

    // CONTROL CONSTANTS

    public final double WHEEL_DIAMETER_INCHES = 4;
    public final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    public final double DRIVE_TICKS_PER_REV = motorTicksPerRev[0];
    public final double DRIVE_GEAR_REDUCTION = 1;
    public final double DRIVE_TICKS_PER_INCH = (((DRIVE_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_INCHES));
    public final double DRIVE_TICKS_PER_DEGREE = ((double)3545.0/360.0); // temp
    public final double DRIVE_STRAFE_CORRECTION = (double)5.0/4.25;
    public final double DISTANCE_BTWN_WHEELS = 12.2047; //temp

    // constructor
    public Drivetrain() { }

    public void initialize(){

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // odo
        resetTicks();
    }

    public void initializeIMU(){

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(imuParameters);
        runtime.reset();
    }

    public void resetTicks(){
        rightPos = rightEncoder.getCurrentPosition();
        leftPos = leftEncoder.getCurrentPosition();
        horzPos = horzEncoder.getCurrentPosition();
    }

    /**
     * Sets each motor to its given power (fl, fr, bl, br) on the interval [-0.1, 0.1]
     */
    public void setPowerAll(double flpower, double frpower, double blpower, double brpower) {

        frontLeft.setPower(flpower);
        frontRight.setPower(frpower);
        backLeft.setPower(blpower);
        backRight.setPower(brpower);
    }

    // Utility methods

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

    public void setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    /**
     * read gamepad inputs and act accordingly
     * used in TeleOp
     */
    public void drive(Telemetry telemetry, Gamepad gamepad1) {

        // FIELD-CENTRIC DRIVE -----------------------------------------------------------------

        // Display rotation data
        telemetry.addData("FC Rotation (Radians): ", getHeading(true));
        telemetry.addData("FC Rotation (Degrees): ", Math.toDegrees(getHeading(true)));
        telemetry.addData("Normal Rotation (Radians): ", Math.toRadians(getHeading(false)));
        telemetry.addData("Normal Rotation (Degrees): ", getHeading(false));

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

        // math
        if (getHeading(true) < 0) {       // If theta is measured clockwise from zero reference

            temp = forward * Math.cos(getHeading(true)) + right * Math.sin(-getHeading(true));
            right = -forward * Math.sin(-getHeading(true)) + right * Math.cos(getHeading(true));
            forward = temp;
        }

        if (getHeading(true) >= 0) {    // If theta is measured counterclockwise from zero reference

            temp = forward * Math.cos(getHeading(true)) - right * Math.sin(getHeading(true));
            right = forward * Math.sin(getHeading(true)) + right * Math.cos(getHeading(true));
            forward = temp;
        }

        // assign calculated values to the power variables
        flpower = forward + right + clockwise;
        frpower = forward - right - clockwise;
        blpower = forward - right + clockwise;
        brpower = forward + right - clockwise;

        // if you have the testing time, maybe remove this one day and see if it causes any
        // problems?
        // Find the maximum of the powers
        double max = Math.max(  Math.max(Math.abs(flpower), Math.abs(frpower)),
                Math.max(Math.abs(blpower), Math.abs(brpower))  );
        // Use this to make sure no motor powers are above 1 (the max value the motor can accept)
        if (max > 1) {

            flpower /= max;
            frpower /= max;
            blpower /= max;
            brpower /= max;
        }

        // Motor powers are set to the power of 3 so that the drivetrain motors accelerates
        // exponentially instead of linearly
        // Note: you may consider, in the future, moving this code block to before the
        // max > 1 code block to see if that is better or worse performance, but I think
        // it will be worse because it may mess up proportions
        flpower = Math.pow(flpower, 3);
        blpower = Math.pow(blpower, 3);
        frpower = Math.pow(frpower, 3);
        brpower = Math.pow(brpower, 3);

        // Motor Power is decreased while the right trigger is held down to allow for more
        // precise robot control
        if (gamepad1.right_trigger > 0.8) {

            flpower /= 3;
            frpower /= 3;
            blpower /= 3;
            brpower /= 3;
        }

        // If the trigger is held down, but not pressed all the way down, motor power will
        // slow down proportionally to how much the trigger is pressed
        else if (gamepad1.right_trigger > 0.1) {

            double driveSlow = -0.8 * gamepad1.right_trigger + 1;

            flpower *= driveSlow;
            frpower *= driveSlow;
            blpower *= driveSlow;
            brpower *= driveSlow;
        }

            /*
            // Alternate version of drive slowing
            // This version does not scale proportionally to the press, but uses a constant
            // multiplier instead
            // Programmers may choose to use this version of drive slowing instead due to a
            // driver's preference
            if (gamepad1.right_trigger > 0.5){

                flpower /= 3;
                blpower /= 3;
                frpower /= 3;
                brpower /= 3;
            }
            */

        setPowerAll(flpower, frpower, blpower, brpower);

        //odo and print info
        updatePos(telemetry);

    }

    public void updatePos(Telemetry telemetry){

        leftChange = getLeftTicks() / DRIVE_TICKS_PER_INCH;
        rightChange = getRightTicks() / DRIVE_TICKS_PER_INCH;
        horzChange = getHorzTicks() / DRIVE_TICKS_PER_INCH;
        x  += (((leftChange + rightChange) / 2.0)) * Math.cos(odoAngle);
        y  += (((leftChange + rightChange) / 2.0)) * Math.sin(odoAngle);
        odoAngle  += (leftChange - rightChange) / DISTANCE_BTWN_WHEELS;
        /*
        x  += (((leftChange + rightChange) / 2.0)) * Math.sin(odoAngle) + horzChange*Math.cos(odoAngle);
        y  += (((leftChange + rightChange) / 2.0)) * Math.cos(odoAngle) + horzChange*Math.sin(odoAngle);
        */
        resetTicks();

        // telemetry
        telemetry.addData("Left encoder ticks", getLeftTicks());
        telemetry.addData("Right encoder ticks", getRightTicks());
        telemetry.addData("Horizontal encoder ticks", getHorzTicks());

        telemetry.addData("X Pos", x);
        telemetry.addData("Y Pos", y);

        telemetry.addData("Theta", odoAngle);
        telemetry.update();

        //idle()
    }

    public double getLeftTicks(){
        return leftEncoder.getCurrentPosition() - leftPos;
    }
    public double getRightTicks(){
        return rightEncoder.getCurrentPosition() - rightPos;
    }
    public double getHorzTicks(){
        return horzEncoder.getCurrentPosition() - horzPos;
    }

    public double getHeading(boolean isFieldCentric) {

        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        angle = orientation.thirdAngle; // temp

        if (isFieldCentric) {
            return Math.toRadians(angle);

        } else {
            // Convert -180 to 180 into 0-360
            if (angle > 0)
                return angle;
            else if (angle < 0)
                return (angle + 360);
            else
                return 0;
        }
    }
}
