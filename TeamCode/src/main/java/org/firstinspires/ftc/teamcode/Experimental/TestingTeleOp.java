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
    }
}