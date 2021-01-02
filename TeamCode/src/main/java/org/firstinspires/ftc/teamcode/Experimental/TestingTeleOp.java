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
    Updated for vision testing
 */

@TeleOp(name = "Test: TeleOp")
public class TestingTeleOp extends TestingSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        telemetry.setAutoClear(false);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); // Enables RGB565 format for image
        vision.vuforia.setFrameQueueCapacity(1); // Store only one frame at a time

        waitForStart();

        while (opModeIsActive()) {

            // Capture Vuforia Frame
            if (gamepad1.a && constants.a == 0) {
                constants.a = 1;

            } else if (!gamepad1.a && constants.a == 1) {
                vision.captureFrame();
                constants.a = 0;
            }

            if (vision.rgbImage != null) {

                // Transpose frame into bitmaps
                vision.setBitmaps();

                // Save bitmaps
                vision.saveBitmap("Bitmap", vision.bitmap);
                vision.saveBitmap("CroppedBitmap", vision.croppedBitmap);

                // Find stack height
                scanBitmap();

                telemetry.addLine();
                telemetry.addData("Stack Height", vision.getStackHeight());
                telemetry.update();
            }
        }
    }
}