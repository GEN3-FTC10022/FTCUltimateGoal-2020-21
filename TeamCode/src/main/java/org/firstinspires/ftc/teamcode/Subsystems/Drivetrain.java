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

    /**
     * Constructs a drive train object.
     */
    public Drivetrain() { }

    /**
     * Contains the drive mode of the object as either field- or robot-centric, to determine
     * TeleOp joystick interpretation.
     */
    public enum DriveMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }

    /**
     * Initializes the drive train by reversing the frontRight and backRight motors, setting the
     * drive train's ZeroPowerBehavior to BRAKE, setting IMU parameters to degrees and m/s/s, and
     * setting driveMode to FIELD_CENTRIC.
     */
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
        //resetTicks();
    }

    /**
     * @deprecated Not in use, may need updates.
     * Performs the math necessary to obtain field-centric horizontal and vertical values.
     *
     * @param vertical the y axis left joystick input
     * @param horizontal the x axis left joystick input
     * @param rotation the x axis right joystick input, used to rotate the bot
     */
    public void applyFieldCentricConversion(double vertical, double horizontal, double rotation) {
        // Math
        if (getHeading(AngleUnit.RADIANS) < 0) {       // If theta is measured clockwise from zero reference

            temp = vertical * Math.cos(getHeading(AngleUnit.RADIANS)) + horizontal * Math.sin(-getHeading(AngleUnit.RADIANS));
            horizontal = -vertical * Math.sin(-getHeading(AngleUnit.RADIANS)) + horizontal * Math.cos(getHeading(AngleUnit.RADIANS));
            vertical = temp;
        }

        if (getHeading(AngleUnit.RADIANS) >= 0) {    // If theta is measured counterclockwise from zero reference

            temp = vertical * Math.cos(getHeading(AngleUnit.RADIANS)) - horizontal * Math.sin(getHeading(AngleUnit.RADIANS));
            horizontal = vertical * Math.sin(getHeading(AngleUnit.RADIANS)) + horizontal * Math.cos(getHeading(AngleUnit.RADIANS));
            vertical = temp;
        }
    }

    /**
     * Updates the x, y, and odoAngle variables for odometry localization to determine the
     * robot's current displacement from its starting position.
     */
    public void updatePosition() {

        leftChange = getLeftTicks() / DRIVE_TICKS_PER_INCH;
        rightChange = getRightTicks() / DRIVE_TICKS_PER_INCH;
        horzChange = getHorzTicks() / DRIVE_TICKS_PER_INCH;

        x  += (((leftChange + rightChange) / 2.0)) * Math.cos(odoAngle);
        y  += (((leftChange + rightChange) / 2.0)) * Math.sin(odoAngle);
        odoAngle  += (leftChange - rightChange) / DRIVE_TRACK_WIDTH;

        // alternate math that didn't seem to make sense
        /*
        x  += (((leftChange + rightChange) / 2.0)) * Math.sin(odoAngle) + horzChange*Math.cos(odoAngle);
        y  += (((leftChange + rightChange) / 2.0)) * Math.cos(odoAngle) + horzChange*Math.sin(odoAngle);
        */

        resetTicks();
    }

    /**
     * Sets the current positions (left, right, & horizontal) to the respective odometers' current
     * positions.
     */
    public void resetTicks(){
        rightPos = rightEncoder.getCurrentPosition();
        leftPos = leftEncoder.getCurrentPosition();
        horzPos = horzEncoder.getCurrentPosition();
    }

    /**
     * Changes all drive train motors to STOP_AND_RESET_ENCODER, which stops the drive train motors
     * and sets their current position to zero.
     */
    public void resetDriveEncoders() {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Sets all drive motors to reach a target position.
     *
     * @param dist the distance, in ticks, for the robot to travel
     * @param fl the modifier for the front left motor (usually 1 or -1 to show direction)
     * @param fr the modifier for the front right motor (usually 1 or -1 to show direction)
     * @param bl the modifier for the back left motor (usually 1 or -1 to show direction)
     * @param br the modifier for the back right motor (usually 1 or -1 to show direction)
     */
    public void setDriveTarget(double dist, double fl, double fr, double bl, double br) {

        frontLeft.setTargetPosition((int) (fl * dist));
        backLeft.setTargetPosition((int) (bl * dist));
        frontRight.setTargetPosition((int) (fr * dist));
        backRight.setTargetPosition((int) (br * dist));
    }

    /**
     * Sets all motors in the drive train to RUN_TO_POSITION RunMode.
     */
    public void setDriveRunMode() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Determines if all the motors in the drive train are busy, where busy means that they are
     * moving towards a set target.
     *
     * @return whether or not all the motors are busy
     */
    public boolean driveIsBusy() {

        if (frontLeft.isBusy() && backLeft.isBusy() && frontRight.isBusy() && backRight.isBusy())
            return true;
        else
            return false;
    }

    /**
     * Sets all motors in the drive train to RUN_USING_ENCODER RunMode.
     */
    public void resetDriveMode() {

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets all motors in the drive train to a specified ZeroPowerBehavior.
     *
     * @param behavior the ZeroPowerBehavior that will be applied to each drive train motor
     */
    public void setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    /**
     * Sets each motor to its respective power (fl, fr, bl, br) on the interval [-0.1, 0.1].
     *
     * @param flpower the power to be assigned to the front left motor
     * @param frpower the power to be assigned to the front right motor
     * @param blpower the power to be assigned to the back left motor
     * @param brpower the power to be assigned to the back right motor
     */
    public void setDrivePower(double flpower, double frpower, double blpower, double brpower) {
        frontLeft.setPower(flpower);
        frontRight.setPower(frpower);
        backLeft.setPower(blpower);
        backRight.setPower(brpower);
    }

    /**
     * Sets all drive train motors to a specified power, which will be the same power for all motors.
     *
     * @param pow the power to apply to every drive train motor
     */
    public void setDrivePower(double pow) {
        frontLeft.setPower(pow);
        backLeft.setPower(pow);
        frontRight.setPower(pow);
        backRight.setPower(pow);
    }

    /**
     * Determines the change in position of the left encoder since the last recorded left encoder
     * position.
     *
     * @return the position, in ticks, of the left encoder
     */
    public double getLeftTicks(){
        return leftEncoder.getCurrentPosition() - leftPos;
    }

    /**
     * Determines the change in position of the right encoder since the last recorded right encoder
     * position.
     *
     * @return the position, in ticks, of the right encoder
     */
    public double getRightTicks(){
        return rightEncoder.getCurrentPosition() - rightPos;
    }

    /**
     * Determines the change in position of the horizontal encoder since the last recorded horisontal
     * encoder position.
     *
     * @return the position, in ticks, of the horizontal encoder
     */
    public double getHorzTicks(){
        return horzEncoder.getCurrentPosition() - horzPos;
    }

    /**
     * Returns the robot's heading as measured by the REV Hub IMU in euler angles [-180,180) or [-π,π).
     *
     * @param angleUnit The desired units for the robot heading to be expressed in.
     * @return The angle the robot is currently facing, in terms of the specified unit.
     */
    public double getHeading(AngleUnit angleUnit) {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, angleUnit);
        angle = orientation.thirdAngle; // temp
        return angle;
    }

    /**
     * Sets the active drive mode of the robot to either FIELD_CENTRIC or ROBOT_CENTRIC.
     *
     * @param driveMode the desired drive mode of the robot, either FIELD_CENTRIC or ROBOT_CENTRIC
     */
    public void setMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }
}
