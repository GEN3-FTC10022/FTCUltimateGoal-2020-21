package org.firstinspires.ftc.teamcode.Experimental;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;

/**
 * 2/5/21
 * CONTROLS:
 *
 * A:           Intake In/Off
 * B:           Intake Out/Off
 * X:           Reset WobbleMech
 * Y:           Advance WobbleMech
 *
 * Up:          Increase Shooter Velocity
 * Down:        Decrease Shooter Velocity
 * Left:        -
 * Right:       Place Wobble Goal
 *
 * L. Bumper:   Launch Multiple
 * R. Bumper:   Launch Single
 *
 * L. Trigger:  -
 * R. Trigger:  Slow Drive
 *
 * L. Stick:    Omnidirectional Drive
 * R. Stick:    Drive Rotation
 *
 * Start:       -
 * Back:        Switch Drive Mode
 */

@TeleOp(name = "TeleOp: Test")
public class TestingTeleOp extends TestingSuperclass {

    @Override
    public void runOpMode() {

        initialize(false);

        waitForStart();

        telemetry.setAutoClear(true);

        intake.down();

        while (opModeIsActive()) {

            // DRIVETRAIN ==========================================================================

            // Drive
            drive();

            // Switch Modes
            if (gamepad1.back && constants.back == 0)
                constants.back++;
            else if (!gamepad1.back && constants.back == 1) {
                if (drivetrain.driveMode == Drivetrain.DriveMode.FIELD_CENTRIC) {
                    drivetrain.setMode(Drivetrain.DriveMode.ROBOT_CENTRIC);
                } else {
                    drivetrain.setMode(Drivetrain.DriveMode.FIELD_CENTRIC);
                }
                constants.back--;
            }

            // SHOOTER =============================================================================

            shooter.runShooter();

            // Velocity
            if (gamepad1.dpad_up && constants.up == 0)
                constants.up++;
            else if (!gamepad1.dpad_up && constants.up == 1) {
                shooter.increaseVelocity();
                constants.up--;
            } else if (gamepad1.dpad_down && constants.down == 0)
                constants.down++;
            else if (!gamepad1.dpad_down && constants.down == 1) {
                shooter.decreaseVelocity();
                constants.down--;
            }

            // Launching
            if (gamepad1.right_bumper && constants.rBumper == 0)
                constants.rBumper++;
            else if (!gamepad1.right_bumper && constants.rBumper == 1) {
                shootSingle();
                constants.rBumper--;
            } else if (gamepad1.left_bumper && constants.lBumper == 0)
                constants.lBumper++;
            else if (!gamepad1.left_bumper && constants.lBumper == 1) {
                shootAll();
                constants.lBumper--;
            }

            // WOBBLE MECH =========================================================================

            // Arm
            if (gamepad1.y && constants.y == 0) {
                constants.y++;
            } else if (!gamepad1.y && constants.y == 1) { // Aim wobble mech
                aim();
                constants.y++;
            } else if (gamepad1.y && constants.y == 2) {
                constants.y++;
            } else if (!gamepad1.y && constants.y == 3) { // Collect wobble goal
                collect();
                constants.y++;
            } else if (gamepad1.y && constants.y == 4) {
                constants.y++;
            } else if (gamepad1.y && constants.y == 5) { // Drop wobble goal
                drop();
                constants.y = 0;
            } else if (gamepad1.x && constants.x == 0) {
                constants.x++;
            } else if (!gamepad1.x && constants.x == 1) { // Reset wobble mech
                resetWobbleMech();
                constants.y = 0;
                constants.x = 0;
            }

            // Place
            if (gamepad1.dpad_right && constants.right == 0)
                constants.right++;
            else if (!gamepad1.dpad_right && constants.right == 1) {
                place();
                constants.y = 0;
                constants.right--;
            }

            // INTAKE ==============================================================================

            // Rollers
            if (gamepad1.a && constants.a == 0)
                constants.a++;
            else if (!gamepad1.a && constants.a == 1) {
                if (intake.status == Intake.Status.IN)
                    intake.off();
                else
                    intake.in();
                constants.a--;
            } else if (gamepad1.b && constants.b == 0)
                constants.b++;
            else if (!gamepad1.b && constants.b == 1) {
                if (intake.status == Intake.Status.OUT)
                    intake.off();
                else
                    intake.out();
                constants.b--;
            }

            // TELEMETRY ===========================================================================

            displayTeleOpTelemetry();
        }
    }
}