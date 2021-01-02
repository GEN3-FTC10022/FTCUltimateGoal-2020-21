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
    Updated for drive testing
 */

@TeleOp(name = "Test: TeleOp")
public class TestingTeleOp extends TestingSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        telemetry.setAutoClear(true);

        waitForStart();

        while (opModeIsActive()) {

            driveTest(false);

            // Telemetry

            telemetry.addData("Heading (Deg)", drivetrain.getHeading(false));
            telemetry.addLine();

            telemetry.addLine("FL: " + drivetrain.flpower + "\t\t" + "FR: " + drivetrain.frpower);
            telemetry.addLine("BL: " + drivetrain.blpower + "\t\t" + "BR: " + drivetrain.brpower);

            telemetry.update();
        }
    }
}