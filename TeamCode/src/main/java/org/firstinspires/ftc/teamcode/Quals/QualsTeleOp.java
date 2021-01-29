package org.firstinspires.ftc.teamcode.Quals;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;

import java.util.concurrent.TimeUnit;

/*
        Controls:
        A: Intake In/Off
        B: Intake Out/Off
        X: Reset Wobble Mech
        Y: Advance Wobble Mech

        Up: Increase Shooter Speed
        Down: Decrease Shooter Speed
        Left:
        Right: Claw Toggle

        Left Stick X: Left/Right Strafe
        Left Stick Y: Forward/Reverse
        Left Stick Button:
        Right Stick X: Rotate
        Right Stick Y:
        Right Stick Button:

        Left Bumper: Multi-Fire
        Left Trigger:
        Right Bumper: Single-Fire
        Right Trigger: Drive Speed Modifier

        Start:
        Back: Toggle Drive Mode
 */

@TeleOp(name = "Quals TeleOp")
public class QualsTeleOp extends QualsSuperclass {

    @Override
    public void runOpMode() {

        initialize(false);

        waitForStart();

        shooter.setTargetVelocity(shooter.MID_SHOT_VELOCITY);

        while (opModeIsActive()) {

            // TELEMETRY ===========================================================================

            displayTeleOpTelemetry();

            // FIELD-CENTRIC DRIVE =================================================================

            vertical = -gamepad1.left_stick_y;
            horizontal= gamepad1.left_stick_x * drivetrain.DRIVE_STRAFE_CORRECTION; // Correction to counteract imperfect strafing
            rotation = gamepad1.right_stick_x;

            // Joystick deadzones
            if (Math.abs(vertical) < 0.05)
                vertical = 0;
            if (Math.abs(horizontal) < 0.05)
                horizontal= 0;
            if (Math.abs(rotation) < 0.05)
                rotation = 0;

            if (drivetrain.driveMode == Drivetrain.DriveMode.FIELD_CENTRIC) {

                // Math
                if (drivetrain.getHeading(true) < 0) {       // If theta is measured clockwise from zero reference

                    drivetrain.temp = vertical * Math.cos(drivetrain.getHeading(true)) + horizontal * Math.sin(-drivetrain.getHeading(true));
                    horizontal= -vertical * Math.sin(-drivetrain.getHeading(true)) + horizontal * Math.cos(drivetrain.getHeading(true));
                    vertical = drivetrain.temp;
                }

                if (drivetrain.getHeading(true) >= 0) {    // If theta is measured counterclockwise from zero reference

                    drivetrain.temp = vertical * Math.cos(drivetrain.getHeading(true)) - horizontal * Math.sin(drivetrain.getHeading(true));
                    horizontal= vertical * Math.sin(drivetrain.getHeading(true)) + horizontal * Math.cos(drivetrain.getHeading(true));
                    vertical = drivetrain.temp;
                }
            }

            // Assign calculated values to the power variables
            drivetrain.flpower = vertical + horizontal + rotation;
            drivetrain.frpower = vertical - horizontal - rotation;
            drivetrain.blpower = vertical - horizontal + rotation;
            drivetrain.brpower = vertical + horizontal - rotation;

            // Find the greatest motor power
            max = Math.max(Math.max(Math.abs(drivetrain.flpower), Math.abs(drivetrain.frpower)),
                    Math.max(Math.abs(drivetrain.blpower), Math.abs(drivetrain.brpower)));
            // Scale motor powers with the greatest motor power
            drivetrain.flpower /= max;
            drivetrain.frpower /= max;
            drivetrain.blpower /= max;
            drivetrain.brpower /= max;

            // Motor power is decreased proportional to the horizontal trigger value to allow for more
            // precise robot control.
            kSlow = -2.0/3.0 * gamepad1.right_trigger + 1;
            drivetrain.flpower *= kSlow;
            drivetrain.frpower *= kSlow;
            drivetrain.blpower *= kSlow;
            drivetrain.brpower *= kSlow;

            drivetrain.setDrivePower(drivetrain.flpower, drivetrain.frpower, drivetrain.blpower, drivetrain.brpower);

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

            // Launcher
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
        }
    }
}

