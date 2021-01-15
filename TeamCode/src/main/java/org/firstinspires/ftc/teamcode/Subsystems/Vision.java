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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static android.graphics.Bitmap.createBitmap;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Vision {

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
    public VuforiaLocalizer vuforia;

    // This is the webcam we are to use. As with other hardware devices such as motors and
    // servos, this device is identified using the robot configuration tool in the FTC application.

    public WebcamName webcamName;

    public boolean targetVisible;

    // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
    // We can pass Vuforia the handle to a camera preview resource (on the RC phone);
    // If no camera monitor is desired, use the parameter-less constructor instead (commented out below).

    // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    public VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    public VuforiaTrackables targetsUltimateGoal;

    public List<VuforiaTrackable> allTrackables;

    public Image rgbImage = null;
    public VuforiaLocalizer.CloseableFrame closeableFrame = null;
    public Bitmap bitmap = null;
    public Bitmap croppedBitmap = null;

    // Crop Variables
    public final int cropInitialX = 360;
    public final int cropInitialY = 160;
    public final int cropFinalX = 405;
    public final int cropFinalY = 200;
    public final int cropWidth = cropFinalX-cropInitialX; // Tested with 53p width
    public final int cropHeight = cropFinalY-cropInitialY; // Tested with 40p height

    // Detection Constants
    public int check = 0;
    public int ringsDetected = 0;
    public int stackHeight = 0;

    public Vision() { }

    public void initialize() {

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // We also indicate which camera on the RC we wish to use.

        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
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

        // For convenience, gather together all the trackable objects in one easily-iterable collection
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);
    }

    public int getStackHeight() {
        switch(ringsDetected) {
            case 0:
                stackHeight = 0;
                break;
            case 1:
                stackHeight = 1;
                break;
            case 2:
                stackHeight = 4;
                break;
        }
        return stackHeight;
    }

    public void captureFrame() {

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

    public void setBitmaps() {

        // Create bitmap based on image dimensions
        bitmap = createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);

        // Copy pixels to bitmap
        rgbImage.getPixels().rewind();
        bitmap.copyPixelsFromBuffer(rgbImage.getPixels());

        // Create cropped bitmap to focus
        croppedBitmap = createBitmap(bitmap, cropInitialX, cropInitialY, cropWidth, cropHeight);
    }

    public void saveBitmap(String name, Bitmap bMap) {

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
}
