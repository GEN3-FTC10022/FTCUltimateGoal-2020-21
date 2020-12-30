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

@TeleOp(name = "Test: TeleOp")
public class TestingTeleOp extends TestingSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up && up == 0) {
                up = 1;
            } else if (!gamepad1.dpad_up && up == 1) {
                shooter.increasePower();
                // shooter.mShooter.setVelocity(1000);
                up = 0;
            }

            if (gamepad1.dpad_down && down == 0) {
                down = 1;
            } else if (!gamepad1.dpad_down && down == 1) {
                shooter.decreasePower();
                // shooter.mShooter.setVelocity(0);
                down = 0;
            }

            if (gamepad1.right_bumper && rBumper == 0) {
                rBumper = 1;
            } else if (!gamepad1.right_bumper && rBumper == 1) {
                shooter.pushTrigger();
                sleep(400);
                shooter.retractTrigger();
                rBumper = 0;
            }

            telemetry.addLine("Shooter % Velocity: " + shooter.percentVelocity);
            // telemetry.addLine("Shooter Set Velocity: " + shooter.percentVelocity * shooter.SHOOTER_MAX_TICKS_PER_SEC);
            telemetry.addLine("Shooter Raw Velocity: " + shooter.getVelocity());
            telemetry.addLine("Trigger Position: " + shooter.getTriggerPosition());
            telemetry.addLine("Shooter RunMode: " + shooter.mShooter.getMode());
            telemetry.update();
        }
    }
}

