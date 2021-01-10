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
        Right:

        Left Stick X: Left/Right Strafe
        Left Stick Y: Forward/Reverse
        Left Stick Button:
        Right Stick X: Rotate
        Right Stick Y:
        Right Stick Button:

        Left Bumper: Claw Toggle
        Left Trigger:
        Right Bumper: Single-Fire
        Right Trigger: Drive Speed Modifier

        Start:
        Back: Toggle Drive Mode
 */

@TeleOp(name = "TeleOp: Qual 2")
public class QualsTeleOp extends QualsSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();

        while (opModeIsActive()) {

            displayTeleOpTelemetry();
            drive();
            handleToggles();

            // Wobble Mech =========================================================================

            if (gamepad1.y && constants.y == 0) {
                constants.y = 1;
            } else if (!gamepad1.y && constants.y == 1) { // Aim wobble mech
                aim();
                constants.y = 2;
            } else if (gamepad1.y && constants.y == 2) {
                constants.y = 3;
            } else if (!gamepad1.y && constants.y == 3) { // Collect wobble goal
                collect();
                constants.y = 4;
            } else if (gamepad1.y && constants.y == 4) {
                constants.y = 5;
            } else if (gamepad1.y && constants.y == 5) { // Drop wobble goal
                drop();
                constants.y = 0;
            } else if (gamepad1.x && constants.x == 0) {
                constants.x = 1;
            } else if (!gamepad1.x && constants.x == 1) { // Reset wobble mech
                resetWobbleMech();
                constants.y = 0;
                constants.x = 0;
            }
        }
    }

    // Gamepad
    protected void handleToggles() {

        if (!gamepadRateLimit.hasExpired())
            return;

        // Intake ==================================================================================

        if (gamepad1.a) {
            if (intake.status == Intake.Status.IN)
                intake.off();
            else
                intake.in();
            gamepadRateLimit.reset();
        }

        if (gamepad1.b) {
            if (intake.status == Intake.Status.OUT)
                intake.off();
            else
                intake.out();
            gamepadRateLimit.reset();
        }

        // Wobble Mech =============================================================================

        if (gamepad1.left_bumper) {
            if (wobbleMech.getClawPosition() == WobbleMech.ClawPosition.CLOSE)
                wobbleMech.clawOpen();
            else
                wobbleMech.clawClose();
            gamepadRateLimit.reset();
        }

        // Drive ===================================================================================

        if (gamepad1.back) {
            if (drivetrain.driveMode == Drivetrain.DriveMode.FIELD_CENTRIC) {
                drivetrain.setMode(Drivetrain.DriveMode.ROBOT_CENTRIC);
            } else {
                drivetrain.setMode(Drivetrain.DriveMode.FIELD_CENTRIC);
            }
            gamepadRateLimit.reset();
        }

        // Shooter =================================================================================

        if (gamepad1.dpad_up)
            shooter.increasePower();
        if (gamepad1.dpad_down)
            shooter.decreasePower();

        if (gamepad1.right_bumper) {
            shooter.pushTrigger();
            shooter.retractTrigger();
            gamepadRateLimit.reset();
        }
    }
}

