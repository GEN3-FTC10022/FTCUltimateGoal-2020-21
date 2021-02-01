package org.firstinspires.ftc.teamcode.Experimental.SubsystemTesters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Util.Constants;

/**
 * Feb. 1, 2021
 *
 * Tests shooter motors individually (or combined, driver can control)
 *
 * x - launcher1 - second press - off
 * a - launcher2 - second press - off
 * b - both off
 * y - both on
 *
 * dpad up - increase launcher velocity
 * dpad down - decrease launcher velocity
 * this velocity will apply to whichever launcher motors are 'on';
 * any motors turned 'on' after this point will use this velocity
 *
 * rBumper - single shot
 * lBumper - shootAll()
 */

@TeleOp(name = "Subsystems: Shooter Test")
public class TestShooter extends LinearOpMode {

    int state;
    private Constants constants = new Constants();
    private Shooter shooter = new Shooter();

    @Override
    public void runOpMode() {

        initialize();

        telemetry.setAutoClear(true);

        waitForStart();

        doTeleOp();
    }

    private void doTeleOp(){

        while (opModeIsActive()) {

            displayTeleOpTelemetry();

            // Launcher 1 ==========================================================================

            // on
            if (gamepad1.x && constants.x == 0)
                constants.x++;
            else if (!gamepad1.x && constants.x == 1){
                shooter.launcherOne.setVelocity(shooter.getTargetVelocity());
                constants.x++;

                // update state
                if (state == 0) state = 1;
                else if (state == 2) state = 3;
            }
            // off
            else if (gamepad1.x && constants.x == 2)
                constants.x++;
            else if (!gamepad1.x && constants.x == 3){
                shooter.launcherOne.setVelocity(0);
                constants.x = 0;

                // update state
                if (state == 1) state = 0;
                else if (state == 3) state = 2;
            }

            // Launcher 2 ==========================================================================

            // on
            if (gamepad1.a && constants.a == 0)
                constants.a++;
            else if (!gamepad1.a && constants.a == 1){
                shooter.launcherTwo.setVelocity(shooter.getTargetVelocity());
                constants.a++;

                // update state
                if (state == 0) state = 2;
                else if (state == 1) state = 3;
            }
            // off
            else if (gamepad1.a && constants.a == 2)
                constants.a++;
            else if (!gamepad1.a && constants.a == 3){
                shooter.launcherTwo.setVelocity(0);
                constants.a = 0;

                // update state
                if (state == 2) state = 0;
                else if (state == 3) state = 1;
            }

            // Both on & off =======================================================================

            // on
            if (gamepad1.y && constants.y == 0)
                constants.y++;
            else if (!gamepad1.y && constants.y == 1){
                shooter.launcherOne.setVelocity(shooter.getTargetVelocity());
                shooter.launcherTwo.setVelocity(shooter.getTargetVelocity());
                constants.y++;
                // fix individual toggle states
                constants.x = 2;
                constants.a = 2;

                // update state
                state = 3;
            }
            // off
            else if (gamepad1.b && constants.y == 2)
                constants.y ++;
            else if (!gamepad1.b && constants.y == 3){
                shooter.launcherOne.setVelocity(0);
                shooter.launcherTwo.setVelocity(0);
                constants.y = 0;
                // fix individual toggle states
                constants.x = 0;
                constants.a = 0;

                // update state
                state = 0;
            }

            // Velocity ============================================================================

            // up
            if (gamepad1.dpad_up && constants.up == 0)
                constants.up++;
            else if (!gamepad1.dpad_up && constants.up == 1) {
                adjustVelocity(1);
                constants.up--;
            }
            // down
            else if (gamepad1.dpad_down && constants.down == 0)
                constants.down++;
            else if (!gamepad1.dpad_down && constants.down == 1) {
                adjustVelocity(-1);
                constants.down--;
            }

            // Launching ===========================================================================

            if (gamepad1.right_bumper && constants.rBumper == 0)
                constants.rBumper++;
            else if (!gamepad1.right_bumper && constants.rBumper == 1) {
                shootSingle();
                constants.rBumper--;
            } else if (gamepad1.left_bumper && constants.lBumper == 0)
                constants.lBumper++;
            else if (!gamepad1.left_bumper && constants.lBumper == 1) {
                shootAll();
                constants.lBumper--;
            }

        }

    }

    public void doAuto() {

    }

    public void initialize() {

        // Telemetry ===============================================================================
        telemetry.setAutoClear(false);
        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        sleep(500);

        // Shooter =============================================================================
        shooter.launcherOne = (DcMotorEx)hardwareMap.dcMotor.get("launcherOne");
        shooter.launcherTwo = (DcMotorEx)hardwareMap.dcMotor.get("launcherTwo");
        shooter.trigger = hardwareMap.servo.get("trigger");
        shooter.initialize();
        telemetry.addLine("Shooter initialized");
        telemetry.update();
        sleep(500);

        // Display telemetry
        telemetry.setAutoClear(true);
        while(!isStarted())
            displayTeleOpTelemetry();

        state = 0;  // 0 = none, 1 = launcher1 is on, 2= launcher 2 is on, 3 = both are on
    }

    public void displayTeleOpTelemetry() {

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Velocity Motor 1 (ticks/s)", shooter.launcherOne.getVelocity());
        telemetry.addData("Velocity Motor 2 (ticks/s)", shooter.launcherTwo.getVelocity());
        telemetry.addData("Target Velocity (ticks/s)", shooter.getTargetVelocity());
        telemetry.update();
    }

    public void adjustVelocity(double signModifier){

        double newVelocity = shooter.getTargetVelocity() + (shooter.VELOCITY_MODIFIER * signModifier);

        shooter.setTargetVelocity(newVelocity);

        if (state == 3){
            shooter.launcherOne.setVelocity(shooter.getTargetVelocity());
            shooter.launcherTwo.setVelocity(shooter.getTargetVelocity());

        } else if (state == 1){
            shooter.launcherOne.setVelocity(shooter.getTargetVelocity());

        } else if (state == 2){
            shooter.launcherTwo.setVelocity(shooter.getTargetVelocity());
        }
    }

    // shooter methods =============================================================================

    public void shootSingle() {
        shooter.pushTrigger();
        sleep(100);
        shooter.retractTrigger();
        shooter.ringsLoaded--;
        if (shooter.ringsLoaded == 0)
            shooter.ringsLoaded = 3;
    }

    public void shootAll() {
        for (int i = 0; i < 3; i++) {
            shootSingle();
            sleep(100);
        }
    }
}