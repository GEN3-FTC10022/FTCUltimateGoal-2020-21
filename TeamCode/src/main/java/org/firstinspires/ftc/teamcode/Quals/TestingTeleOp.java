package org.firstinspires.ftc.teamcode.Quals;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import static android.graphics.Bitmap.createBitmap;

@TeleOp(name = "TeleOp: Testing")
public class TestingTeleOp extends QualsSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();

        // shooter.setSpeed(1);

        while (opModeIsActive()) {

            /*

            if (gamepad1.dpad_up && up == 0) {
                up = 1;
            } else if (!gamepad1.dpad_up && up == 1) {
                shooter.increaseSpeed();
                up = 0;
            }

            if (gamepad1.dpad_down && down == 0) {
                up = 1;
            } else if (!gamepad1.dpad_down && down == 1) {
                shooter.decreaseSpeed();
                up = 0;
            }

            if (gamepad1.right_bumper && rBumper == 0) {
                rBumper = 1;
            } else if (!gamepad1.right_bumper && rBumper == 1) {
                shooter.fireTrigger();
                rBumper = 0;
            }

             */

            if (gamepad1.right_bumper && rBumper == 0) {
                rBumper = 1;
            } else if (!gamepad1.right_bumper && rBumper == 1) {
                shooter.pushTrigger();
                rBumper = 2;
            } else if (gamepad1.right_bumper && rBumper == 2) {
                rBumper = 3;
            } else if (!gamepad1.right_bumper && rBumper == 3) {
                shooter.retractTrigger();
                rBumper = 0;
            }

            telemetry.addLine("Shooter % Velocity: " + shooter.percentVelocity);
            telemetry.addLine("Shooter Raw Velocity: " + shooter.getVelocity());
            telemetry.addLine("Trigger Position: " + shooter.getTriggerPosition());
            telemetry.update();
        }
    }
}

