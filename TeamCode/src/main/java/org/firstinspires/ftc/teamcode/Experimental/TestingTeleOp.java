package org.firstinspires.ftc.teamcode.Experimental;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;

@TeleOp(name = "TeleOp: Test")
public class TestingTeleOp extends TestingSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        telemetry.setAutoClear(true);

        waitForStart();

        while (opModeIsActive()) {

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

            handleToggles();

            telemetry.addData("Arm Position:", wobbleMech.getArmPosition());
            telemetry.addData("Claw Position:", wobbleMech.getClawPosition());
            telemetry.update();
        }
    }

    // Gamepad
    protected void handleToggles() {

        if (!gamepadRateLimit.hasExpired())
            return;

        // Intake ----------------------------------------------------------------------------------

        if (gamepad1.left_bumper) {
            if (wobbleMech.getClawPosition() == WobbleMech.ClawPosition.CLOSE)
                wobbleMech.clawOpen();
            else
                wobbleMech.clawClose();
            gamepadRateLimit.reset();
        }
    }
}