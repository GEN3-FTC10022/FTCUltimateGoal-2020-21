package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

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

    // Objects
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public DcMotor leftEncoder, rightEncoder, horzEncoder;
    public DriveMode driveMode;

    // REV IMU
    public BNO055IMU imu;
    private Orientation orientation;
    public float angle;

    // TeleOp variables
    public double temp;
    public double flpower, frpower, blpower, brpower;

    // Odometry variables
    public int leftPos = 0, rightPos = 0, horzPos = 0;
    public double leftChange = 0, rightChange = 0, horzChange = 0;
    public double x = 0, y = 0;
    public double odoAngle = 0;

    // CONSTANTS
    public final double WHEEL_DIAMETER_INCHES = 4;
    public final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    public final double DRIVE_TICKS_PER_REV = motorTicksPerRev[0];
    public final double DRIVE_GEAR_REDUCTION = 1;
    public final double DRIVE_TICKS_PER_INCH = (((DRIVE_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_INCHES));
    public final double DRIVE_TICKS_PER_DEGREE = (double)3600.0/360.0; // temp
    public final double DRIVE_STRAFE_CORRECTION = (double)5.0/4.25;
    public final double DRIVE_TRACK_WIDTH = 12.2047; //temp

    public Drivetrain() { }

    public enum DriveMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }

    public void initialize(){

        // Drive
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(imuParameters);

        // Mode
        driveMode = DriveMode.FIELD_CENTRIC;

        // Odometry
        // resetTicks();
    }

    public void applyFieldCentricConversion(double vertical, double horizontal, double rotation) {
        // Math
        if (getHeading(AngleUnit.RADIANS) < 0) {       // If theta is measured clockwise from zero reference

            temp = vertical * Math.cos(getHeading(AngleUnit.RADIANS)) + horizontal * Math.sin(-getHeading(AngleUnit.RADIANS));
            horizontal= -vertical * Math.sin(-getHeading(AngleUnit.RADIANS)) + horizontal * Math.cos(getHeading(AngleUnit.RADIANS));
            vertical = temp;
        }

        if (getHeading(AngleUnit.RADIANS) >= 0) {    // If theta is measured counterclockwise from zero reference

            temp = vertical * Math.cos(getHeading(AngleUnit.RADIANS)) - horizontal * Math.sin(getHeading(AngleUnit.RADIANS));
            horizontal= vertical * Math.sin(getHeading(AngleUnit.RADIANS)) + horizontal * Math.cos(getHeading(AngleUnit.RADIANS));
            vertical = temp;
        }
    }

    public void updatePosition(Telemetry telemetry) {

        leftChange = getLeftTicks() / DRIVE_TICKS_PER_INCH;
        rightChange = getRightTicks() / DRIVE_TICKS_PER_INCH;
        horzChange = getHorzTicks() / DRIVE_TICKS_PER_INCH;

        x  += (((leftChange + rightChange) / 2.0)) * Math.cos(odoAngle);
        y  += (((leftChange + rightChange) / 2.0)) * Math.sin(odoAngle);
        odoAngle  += (leftChange - rightChange) / DRIVE_TRACK_WIDTH;

        /*
        x  += (((leftChange + rightChange) / 2.0)) * Math.sin(odoAngle) + horzChange*Math.cos(odoAngle);
        y  += (((leftChange + rightChange) / 2.0)) * Math.cos(odoAngle) + horzChange*Math.sin(odoAngle);
        */

        resetTicks();
    }

    public void resetDriveEncoders() {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetTicks(){
        rightPos = rightEncoder.getCurrentPosition();
        leftPos = leftEncoder.getCurrentPosition();
        horzPos = horzEncoder.getCurrentPosition();
    }

    public void setDriveTarget(double dist, double fl, double fr, double bl, double br) {

        frontLeft.setTargetPosition((int) (fl * dist));
        backLeft.setTargetPosition((int) (bl * dist));
        frontRight.setTargetPosition((int) (fr * dist));
        backRight.setTargetPosition((int) (br * dist));
    }

    public void setDriveRunMode() {
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
     * Sets each motor to its given power (fl, fr, bl, br) on the interval [-0.1, 0.1]
     */

    public void setDrivePower(double flpower, double frpower, double blpower, double brpower) {
        frontLeft.setPower(flpower);
        frontRight.setPower(frpower);
        backLeft.setPower(blpower);
        backRight.setPower(brpower);
    }

    public void setDrivePower(double pow) {
        frontLeft.setPower(pow);
        backLeft.setPower(pow);
        frontRight.setPower(pow);
        backRight.setPower(pow);
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

    /**
     * The REV Hub IMU measures heading in euler angles [-180,180) or [-π,π). This method returns
     * the robot heading.
     * @param angleUnit The desired units for the robot heading to be expressed in.
     * @return The angle the robot is currently facing in.
     */
    public double getHeading(AngleUnit angleUnit) {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, angleUnit);
        angle = orientation.thirdAngle; // temp
        return angle;
    }

    /**
     * Sets the active drive mode of the robot
     * @param driveMode
     */
    public void setMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }
}
