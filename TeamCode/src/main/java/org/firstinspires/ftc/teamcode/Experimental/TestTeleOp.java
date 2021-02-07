package org.firstinspires.ftc.teamcode.Experimental;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;

/**
 * CONTROLS:
 *
 * Gamepad 1 -
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
 *
 * Gamepad 2 -
 * A:           Switch Drive Mode
 * B:           Switch Velocity Control Mode
 * X:           Switch Wobble Mech Control Mode
 * Y:           -
 *
 * Up:          -
 * Down:        -
 * Left:        -
 * Right:       -
 *
 * L. Bumper:   -
 * R. Bumper:   -
 *
 * L. Trigger:  -
 * R. Trigger:  -
 *
 * L. Stick:    -
 * R. Stick:    -
 *
 * Start:       -
 * Back:        -
 *
 * Last Updated: 2/5/21
 * Update Log:
 * -
 */

@TeleOp(name = "TeleOp: Test")
public class TestTeleOp extends TestSuperclass {

    @Override
    public void runOpMode() {

        initialize(false);

        waitForStart();

        telemetry.setAutoClear(true);

        Intake.drop();

        while (opModeIsActive()) {

            // DRIVETRAIN ==========================================================================

            // Drive
            drive();

            // Switch Modes
            if (gamepad1.back && Constants.back == 0)
                Constants.back++;
            else if (!gamepad1.back && Constants.back == 1) {
                if (drivetrain.driveMode == Drivetrain.DriveMode.FIELD_CENTRIC) {
                    drivetrain.setMode(Drivetrain.DriveMode.ROBOT_CENTRIC);
                } else {
                    drivetrain.setMode(Drivetrain.DriveMode.FIELD_CENTRIC);
                }
                Constants.back--;
            }

            // SHOOTER =============================================================================

            shooter.runShooter();

            // Velocity
            if (gamepad1.dpad_up && Constants.up == 0)
                Constants.up++;
            else if (!gamepad1.dpad_up && Constants.up == 1) {
                shooter.increaseVelocity();
                Constants.up--;
            } else if (gamepad1.dpad_down && Constants.down == 0)
                Constants.down++;
            else if (!gamepad1.dpad_down && Constants.down == 1) {
                shooter.decreaseVelocity();
                Constants.down--;
            }

            // Launching
            if (gamepad1.right_bumper && Constants.rBumper == 0)
                Constants.rBumper++;
            else if (!gamepad1.right_bumper && Constants.rBumper == 1) {
                shootSingle();
                Constants.rBumper--;
            } else if (gamepad1.left_bumper && Constants.lBumper == 0)
                Constants.lBumper++;
            else if (!gamepad1.left_bumper && Constants.lBumper == 1) {
                shootAll();
                Constants.lBumper--;
            }

            // WOBBLE MECH =========================================================================

            // Arm
            if (gamepad1.y && Constants.y == 0) {
                Constants.y++;
            } else if (!gamepad1.y && Constants.y == 1) { // Aim wobble mech
                WobbleMech.aim();
                Constants.y++;
            } else if (gamepad1.y && Constants.y == 2) {
                Constants.y++;
            } else if (!gamepad1.y && Constants.y == 3) { // Collect wobble goal
                WobbleMech.collect();
                Constants.y++;
            } else if (gamepad1.y && Constants.y == 4) {
                Constants.y++;
            } else if (gamepad1.y && Constants.y == 5) { // Drop wobble goal
                WobbleMech.drop();
                Constants.y = 0;
            } else if (gamepad1.x && Constants.x == 0) {
                Constants.x++;
            } else if (!gamepad1.x && Constants.x == 1) { // Reset wobble mech
                WobbleMech.resetWobbleMech();
                Constants.y = 0;
                Constants.x = 0;
            }

            // Place
            if (gamepad1.dpad_right && Constants.right == 0)
                Constants.right++;
            else if (!gamepad1.dpad_right && Constants.right == 1) {
                WobbleMech.place();
                Constants.y = 0;
                Constants.right--;
            }

            // INTAKE ==============================================================================

            // Rollers
            if (gamepad1.a && Constants.a == 0)
                Constants.a++;
            else if (!gamepad1.a && Constants.a == 1) {
                if (Intake.getDirection() == Intake.Direction.IN)
                    Intake.off();
                else
                    Intake.in();
                Constants.a--;
            } else if (gamepad1.b && Constants.b == 0)
                Constants.b++;
            else if (!gamepad1.b && Constants.b == 1) {
                if (Intake.getDirection() == Intake.Direction.OUT)
                    Intake.off();
                else
                    Intake.out();
                Constants.b--;
            }

            // TELEMETRY ===========================================================================

            displayTeleOpTelemetry();
        }
    }
}