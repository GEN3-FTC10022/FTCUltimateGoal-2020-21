package org.firstinspires.ftc.teamcode.Experimental.SubsystemTesters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Subsystem;

/**
 * Gamepad 1 -
 * Up:          Increase Target Setting || Increase Target Velocity
 * Down:        Decrease Target Setting || Decrease Target Velocity
 * L. Bumper:   Launch Multiple
 * R. Bumper:   Launch Single
 * Back:        Switch Shooter Control Mode
 */

@TeleOp (name = "Subsystems: Shooter Test")
public class TestShooter extends LinearOpMode {

    private boolean isAuto;

    @Override
    public void runOpMode() {

        initialize(false);

        Constants.reset();

        waitForStart();

        if (isAuto)
            doAuto();
        else {
            Shooter.setTarget(0);
            Shooter.resetTimer();
            doTeleOp();
        }
    }

    private void doTeleOp(){
        while (opModeIsActive()) {

            Shooter.appendTelemetry(true);
            telemetry.update();

            // Velocity
            if (gamepad1.x && Constants.x == 0)
                Constants.x++;
            else if (!gamepad1.x && Constants.x == 1) {
                Shooter.decreaseTarget();
                Constants.x--;
            } else if (gamepad1.y && Constants.y == 0)
                Constants.y++;
            else if (!gamepad1.y && Constants.y == 1) {
                Shooter.increaseTarget();
                Constants.y--;
            }

            // Launching
            if (gamepad1.right_bumper && Constants.rBumper == 0)
                Constants.rBumper++;
            else if (!gamepad1.right_bumper && Constants.rBumper == 1) {
                Intake.off();
                Shooter.launchAll(2);
                Constants.rBumper--;
            }

            // Rollers
            if (gamepad1.a && Constants.a == 0)
                Constants.a++;
            else if (!gamepad1.a && Constants.a == 1 && !gamepad1.start) {
                if (Intake.getDirection() == Intake.Direction.IN)
                    Intake.off();
                else {
                    Intake.in();
                    Shooter.setTarget(0); // Turn off shooter when intake is running
                }
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
        }

    }

    public void doAuto() {

        Shooter.launchAll(2);
        telemetry.addLine("Finished");
        telemetry.update();

        sleep(30000);
    }

    public void initialize(boolean isAuto) {

        this.isAuto = isAuto;

        telemetry.setAutoClear(false);

        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        sleep(500);

        Subsystem.initialize(hardwareMap, telemetry);

        Shooter.initialize();
        Intake.initialize();

        if (!isAuto) {
            telemetry.setAutoClear(true);
            while(!isStarted()) {
                Shooter.appendTelemetry(true);
                telemetry.update();
            }
        }
    }
}