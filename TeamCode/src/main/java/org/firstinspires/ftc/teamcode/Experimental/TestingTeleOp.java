package org.firstinspires.ftc.teamcode.Experimental;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;

/**
 * Jan. 30, 2021
 * Tests shooter motors individually (or combined, driver can control)
 *
 * x - launcher1 - second press - off
 * a - launcher2 - second press - off
 * b - both off
 * y - both on
 *
 * dpad up - increase launcher velocity
 * dpad down - decrease launcher velocity
 * this velocity will apply to whichever launcher motors are 'on';
 * any motors turned 'on' after this point will use this velocity
 *
 * rBumper - single shot
 * lBumper - shootAll()
 */

@TeleOp(name = "TeleOp: Shooter Test")
public class TestingTeleOp extends TestingSuperclass {

    int state;

    @Override
    public void runOpMode() {

        initialize();
        state = 0;  // 0 = none, 1 = launcher1 is on, 2= launcher 2 is on, 3 = both are on

        waitForStart();

        while (opModeIsActive()) {

            // SHOOTER =============================================================================


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
                constants.x--;
            }

            // Claw
            if (gamepad1.dpad_right && constants.right == 0)
                constants.right++;
            else if (!gamepad1.dpad_right && constants.right == 1) {
                if (wobbleMech.getClawPosition() == WobbleMech.ClawPosition.CLOSE)
                    wobbleMech.clawOpen();
                else
                    wobbleMech.clawClose();
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