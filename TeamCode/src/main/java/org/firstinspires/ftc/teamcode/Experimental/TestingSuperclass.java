package org.firstinspires.ftc.teamcode.Experimental;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import static android.graphics.Bitmap.createBitmap;
import static android.graphics.Bitmap.createScaledBitmap;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.Util.Constants.motorTicksPerRev;

public abstract class TestingSuperclass extends LinearOpMode {

    // ROBOT OBJECTS -------------------------------------------------------------------------------

    // Constants
    public Constants constants = new Constants();

    // Vision
    public Vision vision = new Vision();

    // Controller
    public Deadline gamepadRateLimit = new Deadline(Constants.GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

    // METHODS -------------------------------------------------------------------------------------

    // Robot Initialization
    public void initialize() {

        // Vision ==============================================================================
        vision.webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        vision.initialize();
        telemetry.addLine("Vision initialized");
        telemetry.update();
        sleep(500);
    }

    public void displayTeleOpTelemetry() {

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
}
