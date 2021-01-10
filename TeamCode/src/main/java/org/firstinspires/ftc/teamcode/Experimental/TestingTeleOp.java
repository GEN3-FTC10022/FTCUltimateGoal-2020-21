package org.firstinspires.ftc.teamcode.Experimental;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;

@TeleOp(name = "TeleOp: Test")
public class TestingTeleOp extends TestingSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        telemetry.setAutoClear(true);

        waitForStart();

        while (opModeIsActive()) {

            handleToggles();

            displayTeleOpTelemetry();
        }
    }

    // Gamepad
    protected void handleToggles() {

        if (!gamepadRateLimit.hasExpired())
            return;

        // Shooter =================================================================================

        if (gamepad1.dpad_up)
            shooter.increasePower();
        if (gamepad1.dpad_down)
            shooter.decreasePower();

        if (gamepad1.start) {
            // shooter.launcher.setVelocity(28);
            // shooter.launcher.setPower(0.3);
            shooter.launcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad1.right_bumper) {
            shooter.pushTrigger();
            sleep(375);
            shooter.retractTrigger();
            gamepadRateLimit.reset();
        }
    }
}