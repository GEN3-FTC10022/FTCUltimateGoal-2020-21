package org.firstinspires.ftc.teamcode.Experimental.SubsystemTesters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Subsystem;

/**
 * Gamepad 1 -
 * Up:          Increase Target Setting || Increase Target Velocity
 * Down:        Decrease Target Setting || Decrease Target Velocity
 * L. Bumper:   Launch Multiple
 * R. Bumper:   Launch Single
 * Back:        Switch Velocity Control Mode
 */

@TeleOp(name = "Subsystems: Shooter Test")
public class TestShooter extends LinearOpMode {

    private boolean isAuto;

    @Override
    public void runOpMode() {

        initialize(false);

        Constants.reset();

        waitForStart();

        if (isAuto)
            doAuto();
        else
            doTeleOp();
    }

    private void doTeleOp(){
        while (opModeIsActive()) {

            Shooter.appendTelemetry(true);
            telemetry.update();

            Shooter.runLauncher();

            // Velocity Control Mode
            if (gamepad1.back && Constants.back == 0)
                Constants.back++;
            else if (!gamepad1.back && Constants.back == 1) {
                if (Shooter.getVelocityControlMode() == Shooter.VelocityControlMode.PRESET)
                    Shooter.setVelocityControlMode(Shooter.VelocityControlMode.MANUAL);
                else if (Shooter.getVelocityControlMode() == Shooter.VelocityControlMode.MANUAL)
                    Shooter.setVelocityControlMode(Shooter.VelocityControlMode.PRESET);
                Constants.back--;
            }

            // Velocity
            if (gamepad1.dpad_up && Constants.up == 0)
                Constants.up++;
            else if (!gamepad1.dpad_up && Constants.up == 1) {
                Shooter.increaseVelocity();
                Constants.up--;
            } else if (gamepad1.dpad_down && Constants.down == 0)
                Constants.down++;
            else if (!gamepad1.dpad_down && Constants.down == 1) {
                Shooter.decreaseVelocity();
                Constants.down--;
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
        }

    }

    public void doAuto() {
        Shooter.setTarget(4);
        Shooter.runLauncher();
        sleep(2000);
        Shooter.shootAll();
        Shooter.setTarget(0);
        sleep(30000);
    }

    public void initialize(boolean isAuto) {

        this.isAuto = isAuto;

        telemetry.setAutoClear(false);

        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        sleep(500);

        Subsystem.initialize(hardwareMap, telemetry);

        Shooter.initialize("launcherOne", "launcherTwo", "trigger");

        if (!isAuto) {
            telemetry.setAutoClear(true);
            while(!isStarted()) {
                Shooter.appendTelemetry(true);
                telemetry.update();
            }
        }
    }
}