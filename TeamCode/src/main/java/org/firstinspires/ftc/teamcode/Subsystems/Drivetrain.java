package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Util.Subsystem;

import static org.firstinspires.ftc.teamcode.Util.Constants.motorTicksPerRev;

public abstract class Drivetrain extends Subsystem {

    // Devices
    private static DcMotorEx fl, fr, bl, br;
    private static final String HM_FL = "FL";
    private static final String HM_FR = "FR";
    private static final String HM_BL = "BL";
    private static final String HM_BR = "BR";
    private static DcMotor lEncoder, rEncoder, hEncoder;
    private static ControlMode controlMode;

    // REV IMU
    private static BNO055IMU imu;
    private static final String HM_IMU = "imu";
    private static Orientation orientation;
    private static double heading;
    private static double headingZeroCorrection; // Initial Angle Correction

    // Odometry Variables
    private static int leftPos = 0, rightPos = 0, horzPos = 0;
    private static double leftChange = 0, rightChange = 0, horzChange = 0;
    private static double x = 0, y = 0;
    private static double odoAngle = 0;

    // Constants
    private static final double WHEEL_DIAMETER_INCHES = 4;
    private static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    private static final double TICKS_PER_REV = motorTicksPerRev[0];
    private static final double GEAR_REDUCTION = 1;
    private static final double TICKS_PER_INCH = (((TICKS_PER_REV * GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_INCHES));
    private static final double TICKS_PER_DEGREE = 3585.0 /360.0; // temp
    private static final double TRACK_WIDTH = 12.2047; //temp
    public static double STRAFE_CORRECTION = 5.0 /4.25;
    private static final double normalPower = 0.8;
    private static final double strafePower = 0.5;
    private static final double rotatePower = 0.5;

    /**
     * Initializes the drive train by reversing the frontRight and backRight motors, setting the
     * drive train's ZeroPowerBehavior to BRAKE, setting IMU parameters to degrees and m/s/s, and
     * setting driveMode to FIELD_CENTRIC.
     */
    public static void initialize(boolean isAuto) {

        // Hardware Map
        fl = hm.get(DcMotorEx.class, HM_FL);
        fr = hm.get(DcMotorEx.class, HM_FR);
        bl = hm.get(DcMotorEx.class, HM_BL);
        br = hm.get(DcMotorEx.class, HM_BR);
        imu = hm.get(BNO055IMU.class, HM_IMU);

        // Drive
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        if (isAuto) setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        else setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // IMU
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(imuParameters);
        headingZeroCorrection = 0;

        // Control Mode
        controlMode = ControlMode.ROBOT_CENTRIC;

        // Odometry
        //updateTicks();

        tm.addLine("Drivetrain initialized");
        tm.update();
        sleep(500);
    }

    /**
     * Contains the drive mode of the object as either field- or robot-centric, to determine
     * TeleOp joystick interpretation.
     */
    public enum ControlMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }

    /**
     * Sets the active control mode of the robot to the desired control mode
     * @param controlMode the desired control mode of the robot
     * @see Drivetrain.ControlMode
     */
    public static void setControlMode(ControlMode controlMode) {
        Drivetrain.controlMode = controlMode;
    }

    /**
     * Returns the active drive control mode.
     * @return The active drive control mode.
     * @see Drivetrain.ControlMode
     */
    public static ControlMode getControlMode() {
        return controlMode;
    }

    /**
     * Updates the x, y, and odoAngle variables for odometry localization to determine the
     * robot's current displacement from its starting position.
     */
    private static void updatePosition() {

        leftChange = getLeftTicks() / TICKS_PER_INCH;
        rightChange = getRightTicks() / TICKS_PER_INCH;
        horzChange = getHorzTicks() / TICKS_PER_INCH;

        x  += (((leftChange + rightChange) / 2.0)) * Math.cos(odoAngle);
        y  += (((leftChange + rightChange) / 2.0)) * Math.sin(odoAngle);
        odoAngle  += (leftChange - rightChange) / TRACK_WIDTH;

        // alternate math that didn't seem to make sense
        /*
        x  += (((leftChange + rightChange) / 2.0)) * Math.sin(odoAngle) + horzChange*Math.cos(odoAngle);
        y  += (((leftChange + rightChange) / 2.0)) * Math.cos(odoAngle) + horzChange*Math.sin(odoAngle);
        */

        updateTicks();
    }

    /**
     * Sets the current positions (left, right, & horizontal) to the respective odometers' current
     * positions.
     */
    private static void updateTicks(){
        rightPos = rEncoder.getCurrentPosition();
        leftPos = lEncoder.getCurrentPosition();
        horzPos = hEncoder.getCurrentPosition();
    }

    /**
     * Determines the change in position of the left encoder since the last recorded left encoder
     * position.
     *
     * @return the position, in ticks, of the left encoder
     */
    private static double getLeftTicks(){
        return lEncoder.getCurrentPosition() - leftPos;
    }

    /**
     * Determines the change in position of the right encoder since the last recorded right encoder
     * position.
     *
     * @return the position, in ticks, of the right encoder
     */
    private static double getRightTicks(){
        return rEncoder.getCurrentPosition() - rightPos;
    }

    /**
     * Determines the change in position of the horizontal encoder since the last recorded horisontal
     * encoder position.
     *
     * @return the position, in ticks, of the horizontal encoder
     */
    private static double getHorzTicks(){
        return hEncoder.getCurrentPosition() - horzPos;
    }

    /**
     * Set RunMode of all drivetrain motors to the desired RunMode
     * @param runMode The desired RunMode
     */
    private static void setRunMode(DcMotor.RunMode runMode) {
        fl.setMode(runMode);
        bl.setMode(runMode);
        fr.setMode(runMode);
        br.setMode(runMode);
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
    private static void setTargetPosition(double dist, double fl, double fr, double bl, double br) {

        Drivetrain.fl.setTargetPosition((int) (fl * dist));
        Drivetrain.bl.setTargetPosition((int) (bl * dist));
        Drivetrain.fr.setTargetPosition((int) (fr * dist));
        Drivetrain.br.setTargetPosition((int) (br * dist));
    }

    /**
     * Determines if all the motors in the drive train are busy, where busy means that they are
     * moving towards a set target.
     *
     * @return whether or not all the motors are busy
     */
    private static boolean isBusy() {

        return fl.isBusy() && bl.isBusy() && fr.isBusy() && br.isBusy();
    }

    /**
     * Sets all motors in the drive train to a specified ZeroPowerBehavior.
     *
     * @param behavior the ZeroPowerBehavior that will be applied to each drive train motor
     */
    private static void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        fl.setZeroPowerBehavior(behavior);
        fr.setZeroPowerBehavior(behavior);
        bl.setZeroPowerBehavior(behavior);
        br.setZeroPowerBehavior(behavior);
    }

    /**
     * Sets each motor to its respective power (fl, fr, bl, br) on the interval [-0.1, 0.1].
     *
     * @param flpower the power to be assigned to the front left motor
     * @param frpower the power to be assigned to the front right motor
     * @param blpower the power to be assigned to the back left motor
     * @param brpower the power to be assigned to the back right motor
     */
    public static void setPower(double flpower, double frpower, double blpower, double brpower) {
        fl.setPower(flpower);
        fr.setPower(frpower);
        bl.setPower(blpower);
        br.setPower(brpower);
    }

    /**
     * Sets all drive train motors to a specified power, which will be the same power for all motors.
     *
     * @param pow the power to apply to every drive train motor
     */
    private static void setPower(double pow) {
        fl.setPower(pow);
        bl.setPower(pow);
        fr.setPower(pow);
        br.setPower(pow);
    }

    /**
     * Updates the robot's heading as measured by the REV Hub IMU in euler angles [-180,180) degrees.
     * In TeleOp, only call once in the loop.
     */
    public static void updateHeading() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        heading = orientation.thirdAngle - getHeadingCorrection(AngleUnit.DEGREES);
    }

    /**
     * Returns the last updated heading as measured by the REV Hub IMU in euler angles [-180,180)
     * or [-π,π) in the specified units.
     * @param angleUnit The desired angle unit.
     * @return Last updated heading
     * @see #updateHeading()
     */
    public static double getHeading(AngleUnit angleUnit) {
        if (angleUnit == AngleUnit.DEGREES)
            return heading;
        else {
            return Math.toRadians(heading);
        }
    }

    /**
     * Updates the zero heading correction to the current robot angle in degrees.
     */
    public static void setHeadingCorrection() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        headingZeroCorrection = orientation.thirdAngle;

        tm.addData("Correction", getHeadingCorrection(AngleUnit.DEGREES));
        tm.update();
        sleep(200);
    }

    /**
     * Returns the last updated heading correction.
     * @param angleUnit The desired angle unit.
     * @return Last updated heading correction.
     * @see #setHeadingCorrection()
     */
    private static double getHeadingCorrection(AngleUnit angleUnit) {
        if (angleUnit == AngleUnit.DEGREES)
            return headingZeroCorrection;
        else
            return Math.toRadians(headingZeroCorrection);
    }

    /**
     * Moves in the desired direction at the default power for the desired distance in inches. The
     * direction is determined by the angle in degrees of the 8-direction compass directions given
     * by the unit circle. (eg. 45 is northeast).
     * @param direction Desired direction
     * @param inches Desired distance
     * @see #normalPower
     * @see #strafePower
     */
    public static void move(int direction, double inches) {
        double power;
        double target = inches * TICKS_PER_INCH;
        if (direction == 0 || direction == 180) {
            target *= STRAFE_CORRECTION;
            power = strafePower;
        } else power = normalPower;
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        switch(direction) {
            case 0:
                setTargetPosition(target,
                        1, -1,
                        -1, 1);
                break;
            case 45:
                setTargetPosition(target,
                        1, 0,
                        0, 1);
                break;
            case 90:
                setTargetPosition(target,
                        1, 1,
                        1, 1);
                break;
            case 135:
                setTargetPosition(target,
                        0, 1,
                        1, 0);
                break;
            case 180:
                setTargetPosition(target,
                        -1, 1,
                        1, -1);
                break;
            case 225:
                setTargetPosition(target,
                        -1, 0,
                        0, -1);
                break;
            case 270:
                setTargetPosition(target,
                        -1, -1,
                        -1, -1);
                break;
            case 315:
                setTargetPosition(target,
                        0, -1,
                        -1, 0);
                break;
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Float motors during movement
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (isBusy())
            setPower(power);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Brake motors after movement
        setPower(0);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Moves in the desired direction at the desired power for the desired distance in inches. The
     * direction is determined by the angle in degrees of the 8-direction compass directions given
     * by the unit circle. (eg. 45 is northeast).
     * @param direction Desired direction
     * @param power Desired power
     * @param inches Desired distance
     */
    public static void move(int direction, double power, double inches) {
        double target = inches * TICKS_PER_INCH;
        if (direction == 0 || direction == 180) target *= STRAFE_CORRECTION;
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        switch(direction) {
            case 0:
                setTargetPosition(target,
                        1, -1,
                        -1, 1);
                break;
            case 45:
                setTargetPosition(target,
                        1, 0,
                        0, 1);
                break;
            case 90:
                setTargetPosition(target,
                        1, 1,
                        1, 1);
                break;
            case 135:
                setTargetPosition(target,
                        0, 1,
                        1, 0);
                break;
            case 180:
                setTargetPosition(target,
                        -1, 1,
                        1, -1);
                break;
            case 225:
                setTargetPosition(target,
                        -1, 0,
                        0, -1);
                break;
            case 270:
                setTargetPosition(target,
                        -1, -1,
                        -1, -1);
                break;
            case 315:
                setTargetPosition(target,
                        0, -1,
                        -1, 0);
                break;
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Float motors during movement
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (isBusy())
            setPower(power);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Brake motors after movement
        setPower(0);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Rotates for the desired number of degrees at the default power. CW is negative, CCW is
     * positive.
     * @param deltaAngle Angle to turn.
     * @see #rotatePower
     */
    public static void rotate(double deltaAngle) {
        double target = deltaAngle * TICKS_PER_DEGREE;
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setTargetPosition(target,
                -1, 1,
                -1, 1);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Float motors during movement
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (isBusy())
            setPower(rotatePower);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Brake motors after movement
        setPower(0);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Rotates for the desired number of degrees at the specified power. CW is negative, CCW is
     * positive.
     * @param power Desired power
     * @param deltaAngle Angle to turn.
     */
    public static void rotate(double power, double deltaAngle) {
        double target = deltaAngle * TICKS_PER_DEGREE;
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setTargetPosition(target,
                -1, 1,
                -1, 1);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Float motors during movement
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (isBusy())
            setPower(power);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Brake motors after movement
        setPower(0);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Finds the angle difference between the target and current heading. Rotates by this angle in
     * the opposite direction with {@link #rotate(double)} as the CW and CCW directions are flipped.
     * @param targetHeading Desired heading as measured by the REV Hub IMU in euler angles
     * [-180,180) or [-π,π) where 0 is north.
     */
    public static void rotateTo (double targetHeading) {
        double currentHeading = getHeading(AngleUnit.DEGREES);
        double deltaAngle = targetHeading - currentHeading;
        if (Math.abs(deltaAngle) > 180) {
            deltaAngle = 360 - Math.abs(deltaAngle);
        }
        rotate(-deltaAngle);
    }

    /**
     * @deprecated Experimental
     */
    @Deprecated
    public static void rotateToAngle(double power, double targetAngle, boolean displayInfo) {

        tm.setAutoClear(false);
        double initialAngle = getHeading(AngleUnit.DEGREES);
        double deltaAngle = targetAngle - initialAngle;
        if (Math.abs(deltaAngle) > 180) {
            deltaAngle = 360 - Math.abs(deltaAngle);
        }

        if (displayInfo) {
            tm.addData("Initial Angle", initialAngle);
            tm.addData("Target Angle", targetAngle);
            tm.addData("Delta Angle", deltaAngle);
            tm.update();
        }

        double target = deltaAngle * TICKS_PER_DEGREE;

        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTargetPosition(target,
                -1, 1,
                -1, 1);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (isBusy())
            setPower(power);

        if (Math.abs(targetAngle - getHeading(AngleUnit.DEGREES)) > 0.3) {
            rotateToAngle(power, targetAngle, false);
        }

        setPower(0);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (displayInfo) {
            tm.addData("Final Angle", getHeading(AngleUnit.DEGREES));
            tm.addLine("Rotate Finished");
            tm.update();
        }
    }

    /**
     * Appends Drivetrain data to telemetry. For the drive motors, the data is displayed for the
     * front left motor.
     * @param expanded Shows expanded data for troubleshooting.
     */
    public static void appendTelemetry(boolean expanded) {
        tm.addLine("=== DRIVETRAIN ===");
        tm.addData("Heading", Drivetrain.getHeading(AngleUnit.DEGREES));

        tm.addData("Control Mode", Drivetrain.controlMode);
        tm.addLine();

        if (expanded) {
            tm.addLine("\n:: FL Motor ::");
            tm.addData("Correction", Drivetrain.getHeadingCorrection(AngleUnit.DEGREES));
            tm.addData("Current", fl.getCurrent(CurrentUnit.AMPS));
            tm.addData("Current Alert", fl.getCurrentAlert(CurrentUnit.AMPS));
            tm.addData("Over Current", fl.isOverCurrent());
            tm.addData("Run Mode", fl.getMode());
            tm.addData("Encoder PID", fl.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        }

        tm.addLine();
    }
}
