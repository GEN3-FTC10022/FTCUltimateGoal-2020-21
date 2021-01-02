package org.firstinspires.ftc.teamcode.Experimental;

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
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static android.graphics.Bitmap.createBitmap;
import static android.graphics.Bitmap.createScaledBitmap;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.Util.Constants.motorTicksPerRev;

/*
    Updated for Vision Testing
 */

public abstract class TestingSuperclass extends LinearOpMode {

    // ROBOT OBJECTS -------------------------------------------------------------------------------

    // Constants
    public Constants constants = new Constants();

    // Drivetrain
    public Drivetrain drivetrain = new Drivetrain();

    // Vision
    public Vision vision = new Vision();

    // METHODS -------------------------------------------------------------------------------------

    // Robot Initialization
    public void initialize() {

        // Device Initialization

        // Telemetry
        telemetry.setAutoClear(false);
        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        sleep(500);

        // Vision
        vision.webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        vision.initialize();

        telemetry.addLine("Vision initialized");
        telemetry.update();
        sleep(500);

        // Telemetry
        telemetry.addLine("Robot Initialized");
        telemetry.update();
        sleep(500);
    }

    public void scanBitmap() {

        int[] yPos = {vision.croppedBitmap.getHeight()-vision.ringHeight/2,vision.ringHeight/2}; // Bottom to Top
        int[] xPos = {(vision.croppedBitmap.getWidth()/2)-20,vision.croppedBitmap.getWidth()/2,(vision.croppedBitmap.getWidth()/2)+20}; // Left to Right
        int pixel, r, g, b;

        for (int i = 0; i < yPos.length; i++) {

            for (int j = 0; j < xPos.length; j++) {

                pixel = vision.croppedBitmap.getPixel(xPos[j],yPos[i]);
                r = Color.red(pixel);
                g = Color.green(pixel);
                b = Color.blue(pixel);

                if ((17.5/100.0)*(r+g) > b) {
                    vision.check++;
                }
            }

            if (vision.check >= 2) {
                vision.ringsDetected++;
            }

            vision.check = 0;
        }
    }

    public void scanStarterStack(boolean saveBitmaps) {

        // Capture frame from camera
        vision.captureFrame();
        telemetry.addLine("Frame captured");
        telemetry.update();

        if (vision.rgbImage != null) {

            // Transpose frame into bitmaps
            vision.setBitmaps();
            telemetry.addLine("Frame converted to Bitmaps");
            telemetry.update();

            // Save bitmaps to .png files
            if (saveBitmaps) {
                vision.saveBitmap("Bitmap", vision.bitmap);
                vision.saveBitmap("CroppedBitmap", vision.croppedBitmap);
                telemetry.addLine("Frame converted to Bitmaps");
                telemetry.update();
            }

            // Scan bitmap for starter stack height
            scanBitmap();
            telemetry.addLine("Bitmaps scanned");
            telemetry.update();

            telemetry.addLine();
            telemetry.addData("Stack Height", vision.getStackHeight());
            telemetry.update();
        }
    }
}
