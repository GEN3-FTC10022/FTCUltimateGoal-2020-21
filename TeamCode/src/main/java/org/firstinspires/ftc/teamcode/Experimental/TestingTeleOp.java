package org.firstinspires.ftc.teamcode.Experimental;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.teamcode.Quals.QualsSuperclass;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import static android.graphics.Bitmap.createBitmap;
import static android.graphics.Bitmap.createScaledBitmap;

/*
    Updated for wobble mech testing
 */

@TeleOp(name = "Test: TeleOp")
public class TestingTeleOp extends TestingSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); // Enables RGB565 format for image
        vision.vuforia.setFrameQueueCapacity(1); // Store only one frame at a time

        waitForStart();

        while (opModeIsActive()) {

            // Capture Vuforia Frame
            if (gamepad1.a && constants.a == 0) {
                constants.a = 1;

            } else if (!gamepad1.a && constants.a == 1) {
                try {
                    vision.closeableFrame = vision.vuforia.getFrameQueue().take();
                    long numImages = vision.closeableFrame.getNumImages();

                    for (int i = 0; i < numImages; i++) {

                        if (vision.closeableFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                            vision.rgbImage = vision.closeableFrame.getImage(i);
                            if (vision.rgbImage != null)
                                break;
                        }
                    }

                } catch (InterruptedException exc) {

                } finally {
                    if (vision.closeableFrame != null)
                        vision.closeableFrame.close();
                }
            }

            if (vision.rgbImage != null) {

                // Copy Bitmap from Vuforia Frame
                Bitmap stack = createBitmap(vision.rgbImage.getWidth(), vision.rgbImage.getHeight(), Bitmap.Config.RGB_565);
                vision.rgbImage.getPixels().rewind();
                stack.copyPixelsFromBuffer(vision.rgbImage.getPixels());

                // Find Directory
                String path = Environment.getExternalStorageDirectory().toString();
                FileOutputStream out = null;

                // Save Bitmap to file
                try {
                    File file = new File(path, "Bitmap.png");
                    out = new FileOutputStream(file);
                    stack.compress(Bitmap.CompressFormat.PNG, 100, out);

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

                int pixelX = (int)((stack.getWidth()/2.0-1));
                int pixelY = (int)((stack.getHeight()/2.0-1));
                int pixel = stack.getPixel(pixelX,pixelY);

                int stackWidth = stack.getWidth(), stackHeight = stack.getHeight();
                int cropStartX = 370;
                int cropStartY = 186;
                int cropWidth = 82;
                int cropHeight = 52;

                // Create cropped bitmap to show only stones
                Bitmap croppedStack = createBitmap(stack, cropStartX, cropStartY, cropWidth, cropHeight);

                // Save cropped bitmap to file
                try {
                    File file = new File(path, "CroppedBitmap.png");
                    out = new FileOutputStream(file);
                    croppedStack.compress(Bitmap.CompressFormat.PNG, 100, out);

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

                int[] yPos = {47,8}; // Bottom to Top
                int[] xPos = {14,41,68}; // Left to Right
                int cPixel, r, g, b;

                for (int i = 0; i < yPos.length; i++) {

                    for (int j = 0; j < xPos.length; j++) {

                        cPixel = croppedStack.getPixel(xPos[j],yPos[i]);
                        r = Color.red(cPixel);
                        g = Color.green(cPixel);
                        b = Color.blue(cPixel);

                        if (r-90 > b) {
                            vision.check++;
                        }

                        /*
                        telemetry.addData("j", j);
                        telemetry.addData("Pixel Red", Color.red(cPixel));
                        telemetry.addData("Pixel Green", Color.green(cPixel));
                        telemetry.addData("Pixel Blue", Color.blue(cPixel));
                        telemetry.addData("Boolean", (r-90 > b));
                        telemetry.addData("Check Count", vision.check);
                        telemetry.addLine();
                        telemetry.update();
                         */
                    }

                    if (vision.check >= 2) {
                        vision.ringsDetected++;
                    }

                    vision.check = 0;
                }

                telemetry.addLine();
                telemetry.addData("Stack Height", vision.getStackHeight());
                telemetry.update();

                break;
            }
        }

        sleep(15000);
        stop();
    }
}

