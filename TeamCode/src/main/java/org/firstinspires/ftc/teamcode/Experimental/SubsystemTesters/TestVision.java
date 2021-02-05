package org.firstinspires.ftc.teamcode.Experimental.SubsystemTesters;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;

@TeleOp(name = "Subsystems: Vision Test")
public class TestVision extends LinearOpMode {

    private Constants constants = new Constants();
    private Vision vision = new Vision();

    @Override
    public void runOpMode() {

        initialize(false);

        telemetry.setAutoClear(true);

        waitForStart();

        // doTeleOp();

        doAuto();
    }

    public void doTeleOp() {
        while (opModeIsActive()) {

            // TELEMETRY ===========================================================================

            displayTeleOpTelemetry();
        }
    }

    public void doAuto() {
        vuforiaScanStack(true, false);
        sleep(30000);
    }

    public void initialize(boolean isAuto) {

        // Telemetry ===============================================================================
        telemetry.setAutoClear(false);
        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        sleep(500);

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

        int widthMid = (int)(vision.cropWidth/2.0);
        int[] xPos = {widthMid-9,widthMid-6,widthMid-3,widthMid,widthMid+3,widthMid+6,widthMid+9}; // 7 pixels top to bottom
        int[] yPos = {vision.cropHeight-1,0}; // Ring 1 (Rightmost pixel), Ring 4 (Leftmost pixel)
        int pixel, r, g, b, total;

        for (int i = 0; i < yPos.length; i++) {

            for (int j = 0; j < xPos.length; j++) {

                pixel = vision.croppedBitmap.getPixel(xPos[j],yPos[i]);
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

            if (vision.check >= 4) {
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

    public void vuforiaScanTrackable() {

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        vision.targetsUltimateGoal.activate();

        // Change condition to something else later
        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            vision.targetVisible = false;
            for (VuforiaTrackable trackable : vision.allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    vision.targetVisible = true;
                    break;
                }
            }
            telemetry.update();
        }

        // Disable Tracking when we are done;
        vision.targetsUltimateGoal.deactivate();
    }
}