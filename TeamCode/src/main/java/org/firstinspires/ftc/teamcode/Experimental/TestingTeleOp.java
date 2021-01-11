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

            shooter.runShooter();

            if (gamepad1.dpad_up && constants.up == 0)
                constants.up++;
            else if (!gamepad1.dpad_up && constants.up == 1) {
                shooter.increasePower();
                constants.up--;
            } else if (gamepad1.dpad_down && constants.down == 0)
                constants.down++;
            else if (!gamepad1.dpad_down && constants.down == 1) {
                shooter.decreasePower();
                constants.down--;
            }

            displayTeleOpTelemetry();
        }
    }

    // Gamepad
    protected void handleToggles() {

        if (!gamepadRateLimit.hasExpired())
            return;

        // Shooter =================================================================================

        if (gamepad1.right_bumper) {
            shooter.pushTrigger();
            sleep(375);
            shooter.retractTrigger();
            shooter.ringsLoaded--;
            if (shooter.ringsLoaded == 0)
                shooter.ringsLoaded = 3;
            gamepadRateLimit.reset();
        }

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