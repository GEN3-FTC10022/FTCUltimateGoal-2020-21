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
import org.firstinspires.ftc.robotcore.internal.android.dex.EncodedValueReader;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static android.graphics.Bitmap.createBitmap;
import static android.graphics.Bitmap.createScaledBitmap;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public abstract class QualsSuperclass extends LinearOpMode {

    // ROBOT OBJECTS -------------------------------------------------------------------------------

    // Drivetrain
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Odometry
    public Encoder leftVertical, rightVertical, horizontal;

    // REV Sensors
    public BNO055IMU imu;
    public Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    public float angle = orientation.thirdAngle; // temp
    public double theta;

    public double temp;
    public double flpower, frpower, blpower, brpower;

    // button trackers
    public int x = 0, a = 0;

    // Vuforia
    // IMPORTANT: If you are using a USB WebCam, camera choice "BACK" and phone portrait "false"
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY = "AaMuRa7/////AAABmeeXefeDrEfGtTjiMIEuO2djgL8Uz6M9/NrJ" +
            "CrNousZ9V7tnau7MP3q5eACYGf+HgjNwjsOkV8ERj5yJglYfVjm3W9NBeAEAP18/1TMnFvSY6+dalmccEnnbag" +
            "eBAPAVMBLk5OLCA35uka2sjuLb37/rdMPNJGmSqklqcthb1NuxWzpVe7BZcf2YODtUPWnTHKi5t5s6XKQA5p4T" +
            "u6x73Mha8a6jN7hv/pnvneUoG0N5Eih6gZ1sSXKcGfpqjf1npkJUb4AcMoqYE0DE31kUk+V/N2hjNsg9mQSGw2" +
            "TmXG7Iq15ugKdyFwzgpWf6IueyoTKkwOczEiGALV2lObz+fyFLob4rq6HtpkCpL4gkh4xy";

    // Class Members
    private VuforiaLocalizer vuforia;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */

    WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

    private boolean targetVisible;

    /*
     * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
     * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
     * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
     */
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    VuforiaTrackables targetsUltimateGoal;

    List<VuforiaTrackable> allTrackables;

    public Image rgbImage = null;
    public VuforiaLocalizer.CloseableFrame closeableFrame = null;

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

        // Odometry

        // REV Sensors
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        // Telemetry
        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    public void initializeVuforia() {

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);
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

    // Vision Methods
    public void vuforiaScanTarget() {

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsUltimateGoal.activate();

        // Change condition to something else later
        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    break;
                }
            }

            telemetry.update();
        }

        // Disable Tracking when we are done;
        targetsUltimateGoal.deactivate();
    }

    public void vuforiaScanPixel(boolean saveBitmap) {

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); // Enables RGB565 format for image
        vuforia.setFrameQueueCapacity(1); // Store only one frame at a time

        // Capture Vuforia Frame
        while (rgbImage == null) {

            try {

                closeableFrame = vuforia.getFrameQueue().take();
                long numImages = closeableFrame.getNumImages();

                for (int i = 0; i < numImages; i++) {

                    if (closeableFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {

                        rgbImage = closeableFrame.getImage(i);

                        if (rgbImage != null)
                            break;
                    }
                }

            } catch (InterruptedException exc) {

            } finally {

                if (closeableFrame != null)
                    closeableFrame.close();
            }
        }

        if (rgbImage != null) {

            // Copy Bitmap from Vuforia Frame
            Bitmap quarry = createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);
            quarry.copyPixelsFromBuffer(rgbImage.getPixels());

            // Find Directory
            String path = Environment.getExternalStorageDirectory().toString();
            FileOutputStream out = null;

            // Save Bitmap to file
            if (saveBitmap) {
                try {

                    File file = new File(path, "Bitmap.png");
                    out = new FileOutputStream(file);
                    quarry.compress(Bitmap.CompressFormat.PNG, 100, out);

                } catch (Exception e) {
                    e.printStackTrace();

                } finally {

                    try {
                        if (out != null) {
                            out.flush();
                            out.close();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }

            // Crop Bitmap
            // (0,0) is the top-left corner of the bitmap
            int cropStartX;
            int cropStartY;
            int cropWidth;
            int cropHeight;

            int quarryWidth, quarryHeight;
            quarryWidth = quarry.getWidth();
            quarryHeight = quarry.getHeight();

            cropStartX = (int) (quarryWidth * 20.0 / 69.5);     // x initial | max: 31.5 original 26.0 / 69.5
            cropStartY = (int) (quarryHeight * 13.0 / 39.0);    // y initial | max: 33.0 original 13.0 / 39.0
            cropWidth = (int) (quarryWidth * 38.0 / 69.5);      // delta x
            cropHeight = (int) (quarryHeight * 6.0 / 39.0);     // delta y

            // Create cropped bitmap to show only stones
            quarry = createBitmap(quarry, cropStartX, cropStartY, cropWidth, cropHeight);

            // Save cropped bitmap to file
            if (saveBitmap) {
                try {

                    File file = new File(path, "CroppedBitmap.png");
                    out = new FileOutputStream(file);
                    quarry.compress(Bitmap.CompressFormat.PNG, 100, out);

                } catch (Exception e) {
                    e.printStackTrace();

                } finally {

                    try {
                        if (out != null) {
                            out.flush();
                            out.close();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }

            // Compress bitmap to reduce scan time
            quarry = createScaledBitmap(quarry, 110, 20, true);

            int yPos, xPos, pixel, tempColorSum;
            int bitmapWidth = quarry.getWidth();
            int bitmapHeight = quarry.getHeight();
            int maxColor = 1000;
            double factor = 0;              // moves yPos based on which stone
            double difference = 1.0 / 12;   // moves yPos based on the point on the stone

            yPos = (int) (bitmapHeight / 5);

            // cycles through the three stones furthest from the wall
            for (int currentStone = 4; currentStone <= 6; currentStone++) {

                // cycles through 3 points on the stone
                for (int i = 1; i <= 3; i++) {

                    // set yPosition
                    xPos = (int) (bitmapWidth * (factor + difference * i));

                    // gets the pixel
                    pixel = quarry.getPixel(xPos, yPos);

                    // finds the color sum of that pixel
                    tempColorSum = Color.red(pixel) + Color.green(pixel) + Color.blue(pixel);

                    // if it's the darkest pixel yet, it will set the skyStonePos to the currentStone
                    if (tempColorSum < maxColor) {

                        maxColor = tempColorSum;
                    }
                }

                // updates factor so the next cycle will look at pixels on the next stone
                factor += 1.0 / 3.0;
            }

            /*
            // this is a test Rbg ----------------------------------------------------------------------------------
            // this program takes the Rbg of the pixel by use arrays 
            // then it compare it to a set rbg if true it add one to a value
            // if value 3 =  one stone
            // if value 6 = two stone
            // if value 9 = three stone 
            // if value 12 = 4 stone 
            int[] Yloc = {}; // put 4 locations 
            int[] Xloc = {}; // put 3 locations
            int loc = 0; // this is for the arrays
            int stonenum = 0; //this is for the stong count 
            Color stonecolor = new Color(255,255,255);//this is the stone color it is set to white have to change that 
            for( int i = 0; i < 12; i++){
                pixel = quarry.getPixel(Xloc[loc], Yloc[loc]);// this is not needed 
                Color c1 = new Color(quarry.getRGB(Xloc[loc], Yloc[loc]));// this takes the Rbg of the pixel have to test 
                if(c1.getRGB() == stonecolor.getRGB()){
                    stonenum += 1;
                }
                loc += 1;
            }

            telemetry.update();

             */
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
