package org.firstinspires.ftc.teamcode.Experimental;

import android.graphics.Bitmap;
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

/*
    Updated for wobble mech testing
 */

@TeleOp(name = "Test: TeleOp")
public class TestingTeleOp extends TestingSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a && constants.a == 0) {
                constants.a = 1;

            } else if (!gamepad1.a && constants.a == 1) { // Aim arm
                aim();
                constants.a = 2;

            } else if (gamepad1.a && constants.a == 2) {
                constants.a = 3;

            } else if (!gamepad1.a && constants.a == 3) { // Collect wobble goal
                collect();
                constants.a = 4;

            } else if (gamepad1.a && constants.a == 4) {
                constants.a = 5;

            } else if (gamepad1.a && constants.a == 5) { // Release wobble goal
                drop();
                constants.a = 0;

            } else if (gamepad1.b && constants.b == 0) {
                constants.b = 1;

            } else if (!gamepad1.b && constants.b == 1) { // Reset arm
                reset();
                constants.a = 0;
                constants.b = 0;
            }

            if (gamepad1.y && constants.y == 0) {
                constants.y = 1;
            } else if (!gamepad1.y && constants.y == 1) {
                release();
                constants.y = 0;
            }

            /*

            if (gamepad1.x && constants.x == 0) {
                constants.x = 1;
            } else if (!gamepad1.x && constants.x == 1) {
                wobbleMech.clawOpen();
                constants.x = 2;
            } else if (gamepad1.x && constants.x == 2) {
                constants.x = 3;
            } else if (!gamepad1.x && constants.x == 3) {
                wobbleMech.clawClose();
                constants.x = 0;
            }

             */

            telemetry.addLine("Robot Initialized");
            telemetry.addLine("WM Motor Position: " + wobbleMech.getArmPosition());
            telemetry.addLine("WM Motor RunMode: " + wobbleMech.arm.getMode());
            telemetry.addLine("WM Servo Position: " + wobbleMech.lClaw.getPosition());
            telemetry.update();
        }
    }
}

