package org.firstinspires.ftc.teamcode.Experimental.SubsystemTesters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Subsystem;

/**
 * Gamepad 1 -
 * X:           Reset WobbleMech || Arm Down
 * Y:           Advance WobbleMech || Arm Up
 * Right:       Place Wobble Goal || Claw Open/Close
 * Back:        Switch Wobble Mech Control Mode
 */

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

            // Control Mode
            if (gamepad1.back && Constants.back == 0)
                Constants.back++;
            else if (!gamepad1.back && Constants.back == 1) {
                if (WobbleMech.getControlMode() == WobbleMech.ControlMode.ASSISTED)
                    WobbleMech.setControlMode(WobbleMech.ControlMode.MANUAL);
                else if (WobbleMech.getControlMode() == WobbleMech.ControlMode.MANUAL) {
                    WobbleMech.setControlMode(WobbleMech.ControlMode.ASSISTED);
                    WobbleMech.reset();
                }
                Constants.x = 0;
                Constants.y = 0;
                Constants.right = 0;
                Constants.back--;
            }

            if (WobbleMech.getControlMode() == WobbleMech.ControlMode.ASSISTED) {

                // Procedural Auto-Complete Functions
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
                    WobbleMech.reset();
                    Constants.y = 0;
                    Constants.x = 0;
                }

                // Place Wobble Goal
                if (gamepad1.dpad_right && Constants.right == 0)
                    Constants.right++;
                else if (!gamepad1.dpad_right && Constants.right == 1) {
                    WobbleMech.place();
                    Constants.right--;
                }

            } else if (WobbleMech.getControlMode() == WobbleMech.ControlMode.MANUAL) {

                if (gamepad1.y)
                    WobbleMech.armUp();
                else if (gamepad1.x)
                    WobbleMech.armDown();
                else
                    WobbleMech.armStop();

                // Toggle Claw
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
    }

    public void doAuto() {
        WobbleMech.place();
        sleep(30000);
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