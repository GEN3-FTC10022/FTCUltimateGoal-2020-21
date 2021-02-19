package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Util.Subsystem;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import static android.graphics.Bitmap.createBitmap;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public abstract class Vision extends Subsystem {

    // IMPORTANT: If you are using a USB WebCam, camera choice "BACK" and phone portrait "false"
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY = "AaMuRa7/////AAABmeeXefeDrEfGtTjiMIEuO2djgL8Uz6M9/NrJ" +
            "CrNousZ9V7tnau7MP3q5eACYGf+HgjNwjsOkV8ERj5yJglYfVjm3W9NBeAEAP18/1TMnFvSY6+dalmccEnnbag" +
            "eBAPAVMBLk5OLCA35uka2sjuLb37/rdMPNJGmSqklqcthb1NuxWzpVe7BZcf2YODtUPWnTHKi5t5s6XKQA5p4T" +
            "u6x73Mha8a6jN7hv/pnvneUoG0N5Eih6gZ1sSXKcGfpqjf1npkJUb4AcMoqYE0DE31kUk+V/N2hjNsg9mQSGw2" +
            "TmXG7Iq15ugKdyFwzgpWf6IueyoTKkwOczEiGALV2lObz+fyFLob4rq6HtpkCpL4gkh4xy";

    // Class Members
    private static VuforiaLocalizer vuforia;

    private static int cameraMonitorViewId;
    private static VuforiaLocalizer.Parameters parameters;

    private static Image rgbImage = null;
    private static VuforiaLocalizer.CloseableFrame closeableFrame = null;
    private static Bitmap bitmap = null;
    private static Bitmap croppedBitmap = null;

    // Crop Variables
    private static final int CROP_INITIAL_X = 35;
    private static final int CROP_FINAL_X = 55;
    private static final int CROP_INITIAL_Y = 100;
    private static final int CROP_FINAL_Y = 145;
    private static final int CROP_WIDTH = CROP_FINAL_X - CROP_INITIAL_X;
    private static final int CROP_HEIGHT = CROP_FINAL_Y - CROP_INITIAL_Y;

    // Detection Constants
    private static final int ONE_RING_MAX_PIXELS = 0;
    private static final int ONE_RING_MIN_PIXELS = 0;
    public static int ringsFound = 0;

    private static final Exception BrightnessException = new Exception();

    /**
     * Configures the hardware map and initializes vuforia parameters.
     * @param hmWebcam Webcam hardware map name
     */
    public static void initialize(String hmWebcam) {

        // Hardware Map
        parameters.cameraName = hm.get(WebcamName.class, hmWebcam);

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        cameraMonitorViewId = hm.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hm.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Make sure extended tracking is disabled.
        parameters.useExtendedTracking = false;

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        tm.addLine("Vision initialized");
        tm.update();
        sleep(500);
    }

    /**
     * Captures a single frame in RGB565 pixel format and saves it.
     */
    private static void captureFrame() {

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); // Enables RGB565 format for image
        vuforia.setFrameQueueCapacity(1); // Store only one frame at a time

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
    }

    /**
     * Creates a bitmap and a cropped bitmap from the captured frame and the crop variables.
     */
    private static void setBitmaps() {

        // Create bitmap based on image dimensions
        bitmap = createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);

        // Copy pixels to bitmap
        rgbImage.getPixels().rewind();
        bitmap.copyPixelsFromBuffer(rgbImage.getPixels());

        // Create cropped bitmap to focus
        croppedBitmap = createBitmap(bitmap, CROP_INITIAL_X, CROP_INITIAL_Y, CROP_WIDTH, CROP_HEIGHT);
    }

    /**
     * Stores a bitmap object in the device storage as a png file.
     * @param name Output file name
     * @param bMap Bitmap object to save
     */
    private static void saveBitmap(String name, Bitmap bMap) {

        // Find directory
        String path = Environment.getExternalStorageDirectory().toString();
        FileOutputStream out = null;

        // Save bitmap to .png file
        try {
            File file = new File(path, name+".png");
            out = new FileOutputStream(file);
            bMap.compress(Bitmap.CompressFormat.PNG, 100, out);

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

    /**
     * Cycles between 14 pixels to test for RGB values and determine the number of rings
     * @param showPixelData Shows extended information about individual RGB values for each pixel scan.
     * @deprecated Use {@link #scanBitmapEx(boolean)} instead.
     */
    @Deprecated
    private static void scanBitmap(boolean showPixelData) {

        int widthMid = (int)(CROP_WIDTH/2.0);
        int[] xPos = {widthMid-9,widthMid-6,widthMid-3,widthMid,widthMid+3,widthMid+6,widthMid+9}; // 7 pixels left to right
        int[] yPos = {CROP_HEIGHT-1,0}; // Ring 1 (Bottom pixel), Ring 4 (Top pixel)
        int pixel, r, g, b, total;
        int check = 0;

        for (int i = 0; i < yPos.length; i++) {

            for (int j = 0; j < xPos.length; j++) {

                pixel = croppedBitmap.getPixel(xPos[j],yPos[i]);
                r = Color.red(pixel);
                g = Color.green(pixel);
                b = Color.blue(pixel);
                total = r + g + b;

                // Print per pixel RGB to telemetry
                if (showPixelData) {
                    tm.addData("Red", r);
                    tm.addData("Green", g);
                    tm.addData("Blue", b);
                    tm.addData("Total", total);
                    tm.update();
                }

                // If lighting is too bright, increment check if r > g > b with noticeable difference
                // Else if under normal lighting, check if b < 17.5% of r+g
                // Else fail check (too dark)
                if (total > 375) {
                    if (r > g+40 && g > b && b > 70) {
                        tm.addLine("(" + i + "," + j + "): " + "*Bright Check Successful*");
                        tm.addLine();
                        tm.update();
                        check++;
                    } else {
                        tm.addLine("(" + i + "," + j + "): " + "~Bright Check Failed~");
                        tm.addLine();
                        tm.update();
                    }
                } else if (total > 50) {
                    if ((17.5/100.0)*(r+g) > b) {
                        tm.addLine("(" + i + "," + j + "): " + "*Normal Check Successful*");
                        tm.addLine();
                        tm.update();
                        check++;
                    } else {
                        tm.addLine("(" + i + "," + j + "): " + "~Normal Check Failed~");
                        tm.addLine();
                        tm.update();
                    }
                } else {
                    tm.addLine("(" + i + "," + j + "): " + "~Too Dark, Check Failed~");
                    tm.addLine();
                    tm.update();
                }
            }

            tm.addLine();
            tm.addData("Ring " + (i+1) + " Check Count",check + "/7");
            tm.addLine();
            tm.update();

            if (check >= 4)
                ringsFound++;

            check = 0;
        }
    }
    
    private static void scanBitmapEx(boolean showPixelData) {
        int targetPixels = 0;
        int pixel, r, g, b, total;
        int xPos = 0;

        for (int i = 0; i < CROP_HEIGHT; i+=2) {

            // Set pixel and get RGB values
            pixel = croppedBitmap.getPixel(xPos,i);
            r = Color.red(pixel);
            g = Color.green(pixel);
            b = Color.blue(pixel);
            total = r + g + b;

            // Print per pixel RGB to telemetry
            if (showPixelData) {
                tm.addData("Red", r);
                tm.addData("Green", g);
                tm.addData("Blue", b);
                tm.addData("Total", total);
                tm.update();
            }

            if (isTargetPixel(total, r, g, b))
                targetPixels++;
        }
        
        tm.addData("Target Pixels",targetPixels);
        tm.addLine();
        tm.update();

        /*
        if (targetPixels > ONE_RING_MAX_PIXELS)
            ringsFound = 4;
        else if (targetPixels < ONE_RING_MIN_PIXELS)
            ringsFound = 0;
        else
            ringsFound = 1;
         */
    }

    private static boolean isTargetPixel(int total, int r, int g, int b) {

        try {
            if (total < 50)
                throw BrightnessException;
            else if (total > 375) {
                if (r > g+40 && g > b && b > 70) {
                    tm.addLine("Bright Check:\tSuccessful");
                    tm.addLine();
                    tm.update();
                    return true;
                } else {
                    tm.addLine("Bright Check:\tFailed");
                    tm.addLine();
                    tm.update();
                    return false;
                }
            } else {
                if ((17.5/100.0)*(r+g) > b) {
                    tm.addLine("Normal Check:\tSuccessful");
                    tm.addLine();
                    tm.update();
                    return true;
                } else {
                    tm.addLine("Normal Check:\tFailed");
                    tm.addLine();
                    tm.update();
                    return false;
                }
            }

        } catch(Exception e) {
            tm.addLine("Poor lighting condition. Unable to detect pixel.");
            tm.addLine();
            tm.update();
            return false;
        }
    }

    /**
     * Scans the height of the starter stack and updates the number of rings found.
     * @param saveBitmaps Stores the bitmap and the cropped bitmap as png files in the device storage
     * @param showPixelData Shows extended information about individual RGB values for each pixel scan
     */
    public static void vuforiaScanStack(boolean saveBitmaps, boolean showPixelData) {

        tm.setAutoClear(false);

        // Capture frame from camera
        captureFrame();
        tm.addLine("Frame captured");
        tm.addLine();
        tm.update();

        if (rgbImage != null) {

            // Transpose frame into bitmaps
            setBitmaps();
            tm.addLine("Frame converted to bitmaps");
            tm.addLine();
            tm.update();

            // Save bitmaps to .png files
            if (saveBitmaps) {
                saveBitmap("Bitmap", bitmap);
                saveBitmap("CroppedBitmap", croppedBitmap);
                tm.addLine("Bitmaps saved to device");
                tm.addLine();
                tm.update();
            }

            // Scan bitmap for starter stack height
            scanBitmapEx(false);
            tm.addLine("Bitmap scan finished");
            tm.addData("Num Rings", ringsFound);
            tm.update();
        }
    }
}
