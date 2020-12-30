package org.firstinspires.ftc.teamcode.Experimental;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static android.graphics.Bitmap.createBitmap;
import static android.graphics.Bitmap.createScaledBitmap;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.Util.Constants.motorTicksPerRev;

/*
    Updated for Wobble Mech Testing
 */

public abstract class TestingSuperclass extends LinearOpMode {

    // ROBOT OBJECTS -------------------------------------------------------------------------------

    // Constants
    public Constants constants = new Constants();

    // Shooter
    public WobbleMech wobbleMech = new WobbleMech();

    // CONTROL CONSTANTS ---------------------------------------------------------------------------

    // METHODS -------------------------------------------------------------------------------------

    // Robot Initialization
    public void initialize() {

        // Device Initialization

        // Telemetry
        telemetry.addLine("Initializing Robot...");
        telemetry.addLine("Load wobble goal and press 'Start'...");
        telemetry.update();

        // Wobble Mech
        wobbleMech.arm = (DcMotorEx)hardwareMap.dcMotor.get("arm");
        wobbleMech.lClaw = hardwareMap.servo.get("lClaw");
        wobbleMech.rClaw = hardwareMap.servo.get("rClaw");
        wobbleMech.initialize();

        while (wobbleMech.initK == 0) {
            if (gamepad1.start && constants.start == 0) {
                constants.start = 1;
            } else if (!gamepad1.start && constants.start == 1) {
                wobbleMech.clawClose();
                sleep(2000);
                wobbleMech.setArmPosition(0, 0.2);
                constants.start = 0;
                wobbleMech.initK = 1;
            }
        }

        // Telemetry
        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    // UTILITY METHODS -----------------------------------------------------------------------------

    // Wobble Goal

    public void aim() {
        wobbleMech.clawOpen();
        sleep(750);
        wobbleMech.setArmPosition(1, 0.4);
    }

    public void collect() {
        wobbleMech.clawClose();
        sleep(750);
        wobbleMech.setArmPosition(0, 0.2);
    }

    public void release() {
        wobbleMech.setArmPosition(1, 0.2);
        sleep(750);
        wobbleMech.clawOpen();
        sleep(750);
        reset();
    }

    public void drop() {
        wobbleMech.setArmPosition(2, 0.2);
        sleep(750);
        wobbleMech.clawOpen();
        sleep(750);
        reset();
    }

    public void reset() {
        wobbleMech.setArmPosition(0, 0.4);
        wobbleMech.clawClose();
    }
}
