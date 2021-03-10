package org.firstinspires.ftc.teamcode.Experimental;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Subsystem;

/**
 * CONTROLS:
 *
 * Gamepad 1 -
 * A:           Intake In/Off
 * B:           Intake Out/Off
 * X:           -
 * Y:           -
 *
 * Up:          -
 * Down:        -
 * Left:        -
 * Right:       -
 *
 * L. Bumper:   Launch Multiple
 * R. Bumper:   Launch Single
 *
 * L. Trigger:  -
 * R. Trigger:  Drive Slow Modifier
 *
 * L. Stick:    Omnidirectional Drive
 * R. Stick:    Drive Rotation
 *
 * Start:       -
 * Back:        Switch Drivetrain Control Mode
 *
 * Gamepad 2 -
 * A:           -
 * B:           Switch Shooter Control Mode
 * X:           Switch WobbleMech Control Mode
 * Y:           -
 *
 * Up:          Advance WobbleMech || Arm Up
 * Down:        Reset WobbleMech || Arm Down
 * Left:        -
 * Right:       Place Wobble Goal || Claw Open/Close
 *
 * L. Bumper:   Decrease Target Setting || Decrease Target Velocity
 * R. Bumper:   Increase Target Setting || Increase Target Velocity
 *
 * L. Trigger:  -
 * R. Trigger:  -
 *
 * L. Stick:    -
 * R. Stick:    -
 *
 * Start:       -
 * Back:        -
 */

@TeleOp(name = "Test: TeleOp")
public class TestTeleOp extends LinearOpMode {

    private double vertical, horizontal, rotation, max, kSlow, temp;
    private double flpower, frpower, blpower, brpower;

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();

        telemetry.setAutoClear(true);

        while (opModeIsActive()) {

            // SHOOTER =============================================================================

            Shooter.refreshLauncher();

            // Velocity
            if (gamepad2.right_bumper && Constants.rBumper2 == 0)
                Constants.rBumper2++;
            else if (!gamepad2.right_bumper && Constants.rBumper2 == 1) {
                Shooter.increaseTarget();
                Constants.rBumper2--;
            } else if (gamepad2.left_bumper && Constants.lBumper2 == 0)
                Constants.lBumper2++;
            else if (!gamepad2.left_bumper && Constants.lBumper2 == 1) {
                Shooter.decreaseTarget();
                Constants.lBumper2--;
            }

            // Launching
            if (gamepad1.right_bumper && Constants.rBumper == 0)
                Constants.rBumper++;
            else if (!gamepad1.right_bumper && Constants.rBumper == 1) {
                Shooter.shootSingle();
                Constants.rBumper--;
            } else if (gamepad1.left_bumper && Constants.lBumper == 0)
                Constants.lBumper++;
            else if (!gamepad1.left_bumper && Constants.lBumper == 1) {
                Shooter.shootAll();
                Constants.lBumper--;
            }

            // WOBBLE MECH =========================================================================

            // Control Mode
            if (gamepad2.x && Constants.x2 == 0)
                Constants.x2++;
            else if (!gamepad2.x && Constants.x2 == 1) {
                if (WobbleMech.getControlMode() == WobbleMech.ControlMode.ASSISTED) {
                    WobbleMech.setControlMode(WobbleMech.ControlMode.MANUAL);
                    WobbleMech.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else if (WobbleMech.getControlMode() == WobbleMech.ControlMode.MANUAL) {
                    WobbleMech.setControlMode(WobbleMech.ControlMode.ASSISTED);
                    WobbleMech.reset();
                }
                Constants.down2 = 0;
                Constants.up2 = 0;
                Constants.right2 = 0;
                Constants.x2--;
            }

            if (WobbleMech.getControlMode() == WobbleMech.ControlMode.ASSISTED) {

                // Procedural Auto-Complete Functions
                if (gamepad2.dpad_up && Constants.up2 == 0) {
                    Constants.up2++;
                } else if (!gamepad2.dpad_up && Constants.up2 == 1) { // Aim wobble mech
                    WobbleMech.aim();
                    Constants.up2++;
                } else if (gamepad2.dpad_up && Constants.up2 == 2) {
                    Constants.up2++;
                } else if (!gamepad2.dpad_up && Constants.up2 == 3) { // Collect wobble goal
                    WobbleMech.collect();
                    Constants.up2++;
                } else if (gamepad2.dpad_up && Constants.up2 == 4) {
                    Constants.up2++;
                } else if (gamepad2.dpad_up && Constants.up2 == 5) { // Drop wobble goal
                    WobbleMech.drop();
                    Constants.up2 = 0;
                } else if (gamepad2.dpad_down && Constants.down2 == 0) {
                    Constants.down2++;
                } else if (!gamepad2.dpad_down && Constants.down2 == 1) { // Reset wobble mech
                    WobbleMech.reset();
                    Constants.up2 = 0;
                    Constants.down2 = 0;
                }

                // Place Wobble Goal
                if (gamepad2.dpad_right && Constants.right2 == 0)
                    Constants.right2++;
                else if (!gamepad2.dpad_right && Constants.right2 == 1) {
                    WobbleMech.place();
                    Constants.right2--;
                }

            } else if (WobbleMech.getControlMode() == WobbleMech.ControlMode.MANUAL) {

                if (gamepad2.dpad_up)
                    WobbleMech.armUp();
                else if (gamepad2.dpad_down)
                    WobbleMech.armDown();
                else
                    WobbleMech.armStop();

                // Toggle Claw
                if (gamepad2.dpad_right && Constants.right2 == 0)
                    Constants.right2++;
                else if (!gamepad2.dpad_right && Constants.right2 == 1) {
                    if (WobbleMech.getClawPosition() == WobbleMech.ClawPosition.CLOSED)
                        WobbleMech.clawOpen();
                    else
                        WobbleMech.clawClose();
                    Constants.right2--;
                }
            }

            // INTAKE ==============================================================================

            // Rollers
            if (gamepad1.a && Constants.a == 0)
                Constants.a++;
            else if (!gamepad1.a && Constants.a == 1 && !gamepad1.start) {
                if (Intake.getDirection() == Intake.Direction.IN)
                    Intake.off();
                else
                    Intake.in();
                Constants.a--;
            } else if (gamepad1.b && Constants.b == 0 && !gamepad1.start)
                Constants.b++;
            else if (!gamepad1.b && Constants.b == 1) {
                if (Intake.getDirection() == Intake.Direction.OUT)
                    Intake.off();
                else
                    Intake.out();
                Constants.b--;
            }

            // DRIVETRAIN ==========================================================================

            Drivetrain.updateHeading();
            vertical = -gamepad1.left_stick_y;
            horizontal = gamepad1.left_stick_x * Drivetrain.STRAFE_CORRECTION; // Correction to counteract imperfect strafing
            rotation = gamepad1.right_stick_x;
            kSlow = -2.0/3.0 * gamepad1.right_trigger + 1;

            // Apply Field-Centric Conversion
            if (Drivetrain.getControlMode() == Drivetrain.ControlMode.FIELD_CENTRIC) {

                // If robot is facing right
                if (Drivetrain.getHeading(AngleUnit.RADIANS) < 0) {
                    temp = vertical * Math.cos(Drivetrain.getHeading(AngleUnit.RADIANS)) + horizontal * Math.sin(-Drivetrain.getHeading(AngleUnit.RADIANS));
                    horizontal = -vertical * Math.sin(-Drivetrain.getHeading(AngleUnit.RADIANS)) + (horizontal * Math.cos(Drivetrain.getHeading(AngleUnit.RADIANS)));
                }

                // If robot is facing left
                else {
                    temp = vertical * Math.cos(Drivetrain.getHeading(AngleUnit.RADIANS)) - (horizontal * Math.sin(Drivetrain.getHeading(AngleUnit.RADIANS)));
                    horizontal = vertical * Math.sin(Drivetrain.getHeading(AngleUnit.RADIANS)) + (horizontal * Math.cos(Drivetrain.getHeading(AngleUnit.RADIANS)));
                }

                vertical = temp;
            }

            // Assign calculated values to the power variables
            flpower = vertical + horizontal + rotation;
            frpower = vertical - horizontal - rotation;
            blpower = vertical - horizontal + rotation;
            brpower = vertical + horizontal - rotation;

            // Find the greatest motor power
            max = Math.max(Math.max(Math.abs(flpower), Math.abs(frpower)),
                    Math.max(Math.abs(blpower), Math.abs(brpower)));
            // Scale motor powers with the greatest motor power
            flpower /= max;
            frpower /= max;
            blpower /= max;
            brpower /= max;

            // Motor power is decreased proportional to the horizontal trigger value to allow for more
            // precise robot control.
            flpower *= kSlow;
            frpower *= kSlow;
            blpower *= kSlow;
            brpower *= kSlow;

            Drivetrain.setPower(flpower, frpower, blpower, brpower);

            // Switch Modes
            if (gamepad1.back && Constants.back == 0)
                Constants.back++;
            else if (!gamepad2.back && Constants.back == 1) {
                if (Drivetrain.getControlMode() == Drivetrain.ControlMode.FIELD_CENTRIC) {
                    Drivetrain.setControlMode(Drivetrain.ControlMode.ROBOT_CENTRIC);
                } else {
                    Drivetrain.setControlMode(Drivetrain.ControlMode.FIELD_CENTRIC);
                }
                Constants.back--;
            }

            // TELEMETRY ===========================================================================

            updateTelemetry();
        }
    }

    private void initialize() {

        Constants.reset();

        telemetry.setAutoClear(false);

        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        Subsystem.initialize(hardwareMap, telemetry);
        WobbleMech.initialize();
        Intake.initialize();
        Shooter.initialize();
        Drivetrain.initialize(false);

        telemetry.addLine("Setting Correction...");
        telemetry.update();

        do {
            Drivetrain.setHeadingCorrection();
            Drivetrain.updateHeading();
            // Break out of loop if initialization is stopped to prevent forced restart
            if (isStopRequested()) {
                break;
            }
        } while (Drivetrain.getHeading(AngleUnit.DEGREES) != 0);

        telemetry.addLine("Initialization Finished");
        telemetry.update();
        sleep(1000);

        // Display telemetry
        telemetry.setAutoClear(true);
        while(!isStarted())
            updateTelemetry();
    }

    private void updateTelemetry() {
        Shooter.appendTelemetry(false);
        WobbleMech.appendTelemetry(false);
        Intake.appendTelemetry(false);
        Drivetrain.appendTelemetry(false);
        telemetry.update();
    }
}