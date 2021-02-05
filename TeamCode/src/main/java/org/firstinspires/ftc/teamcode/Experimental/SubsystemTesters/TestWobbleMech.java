package org.firstinspires.ftc.teamcode.Experimental.SubsystemTesters;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;

@TeleOp(name = "Subsystems: WobbleMech Test")
public class TestWobbleMech extends LinearOpMode {

    private Constants constants = new Constants();
    private WobbleMech wobbleMech = new WobbleMech();

    @Override
    public void runOpMode() {

        initialize(false);

        telemetry.setAutoClear(true);

        waitForStart();

        doTeleOp();

        // doAuto();
    }

    public void doTeleOp() {
        while (opModeIsActive()) {

            // TELEMETRY ===========================================================================

            displayTeleOpTelemetry();

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
                constants.x = 0;
            }

            // Place Temp
            if (gamepad1.left_bumper && constants.lBumper == 0)
                constants.lBumper++;
            else if (!gamepad1.dpad_left && constants.lBumper == 1) {
                place();
                constants.lBumper--;
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

        }
    }

    public void doAuto() {

    }

    public void initialize(boolean isAuto) {

        // Telemetry ===============================================================================
        telemetry.setAutoClear(false);
        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        sleep(500);

        // Wobble Mech =============================================================================
        wobbleMech.arm = (DcMotorEx)hardwareMap.dcMotor.get("arm");
        wobbleMech.lClaw = hardwareMap.servo.get("lClaw");
        wobbleMech.rClaw = hardwareMap.servo.get("rClaw");
        wobbleMech.initialize();
        telemetry.addLine("Wobble Mech initialized");
        telemetry.update();
        sleep(500);

        if (isAuto) {

            telemetry.addLine();
            telemetry.addLine("Load wobble goal and press 'A', or press 'B' to cancel...");
            telemetry.update();

            while (wobbleMech.initK == 0) {

                if (gamepad1.a) {
                    // Set wobble goal to pre-loaded position
                    wobbleMech.clawClose();
                    sleep(1000);
                    wobbleMech.setArmPosition(WobbleMech.ArmPosition.REST);

                    telemetry.addLine("Wobble goal loaded");
                    telemetry.update();

                    wobbleMech.initK++;
                    sleep(500);
                }

                // Cancel wobble goal pre-load
                if (gamepad1.b) {
                    // Reset wobble mech
                    resetWobbleMech();

                    telemetry.addLine("Wobble goal not loaded");
                    telemetry.update();

                    wobbleMech.initK++;
                    sleep(500);
                }

                // Break out of loop if initialization is stopped to prevent forced restart
                if (isStopRequested()) {
                    break;
                }
            }

        } else {
            resetWobbleMech();
        }

        // Display telemetry
        telemetry.setAutoClear(true);
        while(!isStarted())
            displayTeleOpTelemetry();
    }

    public void displayTeleOpTelemetry() {

        telemetry.addLine("=== WOBBLE MECH ===");
        telemetry.addData("Arm Position", wobbleMech.getArmPosition());
        telemetry.addData("Arm RunMOde", wobbleMech.arm.getMode());
        telemetry.addData("Claw Position", wobbleMech.getClawPosition());
        telemetry.update();
    }

    // Wobble Mech Methods =========================================================================

    public void aim() {
        wobbleMech.clawOpen();
        sleep(500);
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.LOW);
    }

    public void collect() {
        wobbleMech.clawClose();
        sleep(500);
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.REST);
    }

    public void place() {
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.LOW);
        sleep(500);
        wobbleMech.clawOpen();
        sleep(500);
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.REST);
    }

    public void drop() {
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.HIGH);
        sleep(500);
        wobbleMech.clawOpen();
        sleep(500);
        resetWobbleMech();
    }

    public void resetWobbleMech() {
        wobbleMech.setArmPosition(WobbleMech.ArmPosition.REST);
        sleep(500);
        wobbleMech.zeroArm();
        wobbleMech.clawClose();
    }
}