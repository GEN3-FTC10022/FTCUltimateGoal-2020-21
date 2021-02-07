package org.firstinspires.ftc.teamcode.Experimental.SubsystemTesters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Subsystem;

@TeleOp(name = "Subsystems: WobbleMech Test")
public class TestWobbleMech extends LinearOpMode {

    private boolean isAuto;

    @Override
    public void runOpMode() {

        initialize(false);

        waitForStart();

        if (isAuto)
            doAuto();
        else
            doTeleOp();
    }

    public void doTeleOp() {
        while (opModeIsActive()) {

            WobbleMech.appendTelemetry(true);
            telemetry.update();

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
            if (gamepad1.left_bumper && Constants.lBumper == 0)
                Constants.lBumper++;
            else if (!gamepad1.dpad_left && Constants.lBumper == 1) {
                WobbleMech.place();
                Constants.lBumper--;
            }

            // Claw
            if (gamepad1.dpad_right && Constants.right == 0)
                Constants.right++;
            else if (!gamepad1.dpad_right && Constants.right == 1) {
                if (WobbleMech.getClawPosition() == WobbleMech.ClawPosition.CLOSED)
                    WobbleMech.clawOpen();
                else
                    WobbleMech.clawClose();
                Constants.right--;
            }

        }
    }

    public void doAuto() {

    }

    public void initialize(boolean isAuto) {

        this.isAuto = isAuto;

        telemetry.setAutoClear(false);

        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        sleep(500);

        Subsystem.initialize(hardwareMap,telemetry);

        WobbleMech.initialize("arm","lClaw","rClaw");

        if (!isAuto) {
            telemetry.setAutoClear(true);
            while(!isStarted()) {
                WobbleMech.appendTelemetry(true);
                telemetry.update();
            }
        }
    }
}