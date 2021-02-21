package org.firstinspires.ftc.teamcode.Experimental.SubsystemTesters;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Subsystem;

@TeleOp(name = "Subsystems: Drivetrain Test")
public class TestDrivetrain extends LinearOpMode {

    private boolean isAuto;

    @Override
    public void runOpMode() {

        initialize(false);

        waitForStart();

        if (isAuto)
            doAuto();
        else
            doTeleOp();
    }

    private void initialize(boolean isAuto) {

        this.isAuto = isAuto;

        Constants.reset();

        telemetry.setAutoClear(false);

        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        sleep(500);

        Subsystem.initialize(hardwareMap,telemetry);

        Drivetrain.initialize("frontLeft","frontRight",
                "backLeft", "backRight", "imu");

        telemetry.addLine("Setting Correction...");
        telemetry.update();
        sleep(500);

        while (Drivetrain.getHeading(AngleUnit.DEGREES) != 0) {
            Drivetrain.setHeadingCorrectionDegrees();
            Drivetrain.updateHeading();
            // Break out of loop if initialization is stopped to prevent forced restart
            if (isStopRequested()) {
                break;
            }
        }

        if (!isAuto) {
            telemetry.setAutoClear(true);
            while(!isStarted()) {
                Drivetrain.appendTelemetry(true);
                telemetry.update();
            }
        }
    }

    private void doTeleOp() {

        double vertical, horizontal, rotation, max, kSlow, temp;
        double flpower, frpower, blpower, brpower;

        while (opModeIsActive()) {

            // Telemetry
            telemetry.setAutoClear(true);
            Drivetrain.appendTelemetry(true);
            telemetry.update();

            // Drive
            Drivetrain.updateHeading();
            vertical = -gamepad1.left_stick_y;
            horizontal = gamepad1.left_stick_x * Drivetrain.STRAFE_CORRECTION; // Correction to counteract imperfect strafing
            rotation = gamepad1.right_stick_x;
            kSlow = -2.0/3.0 * gamepad1.right_trigger + 1;

            // FIELD-CENTRIC DRIVE =====================================================================

            if (Drivetrain.controlMode == Drivetrain.ControlMode.FIELD_CENTRIC) {

                // If robot is facing right
                if (Drivetrain.getHeading(AngleUnit.RADIANS) < 0) {
                    temp = vertical * Math.cos(Drivetrain.getHeading(AngleUnit.RADIANS)) + horizontal * Math.sin(-Drivetrain.getHeading(AngleUnit.RADIANS));
                    horizontal = -vertical * Math.sin(-Drivetrain.getHeading(AngleUnit.RADIANS)) + (horizontal * Math.cos(Drivetrain.getHeading(AngleUnit.RADIANS)));
                }

                // If robot is facing left
                else {
                    temp = vertical * Math.cos(Drivetrain.getHeading(AngleUnit.RADIANS)) - (horizontal * Math.sin(Drivetrain.getHeading(AngleUnit.RADIANS)));
                    horizontal = vertical * Math.sin(Drivetrain.getHeading(AngleUnit.RADIANS)) + (horizontal * Math.cos(Drivetrain.getHeading(AngleUnit.RADIANS)));
                }

                vertical = temp;
            }

            // Assign calculated values to the power variables
            flpower = vertical + horizontal + rotation;
            frpower = vertical - horizontal - rotation;
            blpower = vertical - horizontal + rotation;
            brpower = vertical + horizontal - rotation;

            // Find the greatest motor power
            max = Math.max(Math.max(Math.abs(flpower), Math.abs(frpower)),
                    Math.max(Math.abs(blpower), Math.abs(brpower)));
            // Scale motor powers with the greatest motor power
            flpower /= max;
            frpower /= max;
            blpower /= max;
            brpower /= max;

            // Motor power is decreased proportional to the horizontal trigger value to allow for more
            // precise robot control.
            flpower *= kSlow;
            frpower *= kSlow;
            blpower *= kSlow;
            brpower *= kSlow;

            Drivetrain.setPower(flpower, frpower, blpower, brpower);

            // Switch Modes
            if (gamepad1.back && Constants.back == 0)
                Constants.back++;
            else if (!gamepad1.back && Constants.back == 1) {
                if (Drivetrain.controlMode == Drivetrain.ControlMode.FIELD_CENTRIC) {
                    Drivetrain.setControlMode(Drivetrain.ControlMode.ROBOT_CENTRIC);
                } else {
                    Drivetrain.setControlMode(Drivetrain.ControlMode.FIELD_CENTRIC);
                }
                Constants.back--;
            }

            if (gamepad1.a && Constants.a == 0)
                Constants.a++;
            else if (!gamepad1.a && Constants.a == 1) {
                Drivetrain.rotate(0.6, 180);
                Constants.a--;
            }
        }
    }

    public void doAuto() {
        Drivetrain.rotate(0.6, 90);
        sleep(30000);
    }
}