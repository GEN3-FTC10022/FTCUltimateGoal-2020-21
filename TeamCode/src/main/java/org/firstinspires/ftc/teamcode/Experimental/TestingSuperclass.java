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
    public WobbleMech wobbleMech = new WobbleMech();

    // Controller
    public Deadline gamepadRateLimit = new Deadline(Constants.GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

    // METHODS -------------------------------------------------------------------------------------

    // Robot Initialization
    public void initialize() {

        wobbleMech.arm = (DcMotorEx)hardwareMap.dcMotor.get("arm");
        wobbleMech.lClaw = hardwareMap.servo.get("lClaw");
        wobbleMech.rClaw = hardwareMap.servo.get("rClaw");
        wobbleMech.initialize();
        telemetry.addLine("Wobble Mech initialized");
        telemetry.addLine();

        // telemetry.addLine("Load wobble goal and press 'A' or press 'B' to cancel load");
        // telemetry.addLine();
        telemetry.update();
        sleep(500);

        /*
        while (wobbleMech.initK == 0) {

            if (gamepad1.a) {
                // Set wobble goal to pre-loaded position
                wobbleMech.clawClose();
                sleep(2000);
                wobbleMech.setArmPosition(WobbleMech.ArmPosition.REST, 0.2);

                telemetry.addLine("Wobble goal loaded");
                telemetry.update();

                wobbleMech.initK = 1;
                sleep(500);
            }

            // Cancel wobble goal pre-load
            if (gamepad1.b) {
                // Reset wobble mech
                resetWobbleMech();

                telemetry.addLine("Wobble goal not loaded");
                telemetry.update();

                wobbleMech.initK = 1;
                sleep(500);
            }

            // Break out of loop if initialization is stopped to prevent forced restart
            if (isStopRequested()) {
                break;
            }
        }
         */
    }

    // Wobble Mech Methods =========================================================================

    public void aim() {
        wobbleMech.clawOpen();
        sleep(750);
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.LOW, 0.3);
    }

    public void collect() {
        wobbleMech.clawClose();
        sleep(750);
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.REST, 0.2);
    }

    public void release() {
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.LOW, 0.2);
        sleep(750);
        wobbleMech.clawOpen();
        sleep(750);
        resetWobbleMech();
    }

    public void drop() {
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.HIGH, 0.2);
        sleep(750);
        wobbleMech.clawOpen();
        sleep(750);
        resetWobbleMech();
    }

    public void resetWobbleMech() {
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.REST, 0.3);
        wobbleMech.clawClose();
    }
}
