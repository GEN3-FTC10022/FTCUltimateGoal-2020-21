package org.firstinspires.ftc.teamcode.Quals;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Util.Constants;

import java.util.concurrent.TimeUnit;

/*
        Controls:
        A: Activate Wobble Mech && Pre-load Confirm
        B: Restart Wobble Mech && Pre-load Cancel
        X:
        Y: Zero Wobble Mech

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

        Left Bumper:
        Left Trigger:
        Right Bumper: Single-Fire
        Right Trigger:

        Start:
        Back: Toggle Drive Mode
 */

@TeleOp(name = "TeleOp: Quals")
public class QualsTeleOp extends QualsSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        telemetry.setAutoClear(true);

        waitForStart();

        while (opModeIsActive()) {

            // Telemetry
            telemetry.addData("Heading (Deg)", drivetrain.getHeading(false));
            telemetry.addLine();

            telemetry.addLine("Wobble Mech:");
            telemetry.addData("Arm Position", wobbleMech.armPosition);
            telemetry.addData("Arm RunMode", wobbleMech.arm.getMode());
            telemetry.addData("Claw Position", wobbleMech.clawPosition);
            telemetry.addLine();

            /*
            telemetry.addData("L-Encoder", drivetrain.getLeftTicks());
            telemetry.addData("R-Encoder", drivetrain.getRightTicks());
            telemetry.addData("H-Encoder", drivetrain.getHorzTicks());
            telemetry.addLine();

            telemetry.addData("X Pos", drivetrain.x);
            telemetry.addData("Y Pos", drivetrain.y);
            telemetry.addData("OdoAngle", drivetrain.odoAngle);
            telemetry.addLine();
             */

            telemetry.update();

            drive();

            handleToggles();
        }
    }

    // Gamepad
    protected void handleToggles() {

        if (!gamepadRateLimit.hasExpired())
            return;

        // Intake ----------------------------------------------------------------------------------

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

        // Drive -----------------------------------------------------------------------------------

        if (gamepad1.back) {
            if (drivetrain.driveMode == Drivetrain.DriveMode.FIELD_CENTRIC) {
                drivetrain.setMode(Drivetrain.DriveMode.ROBOT_CENTRIC);
            } else {
                drivetrain.setMode(Drivetrain.DriveMode.FIELD_CENTRIC);
            }
            gamepadRateLimit.reset();
        }
    }
}

