package org.firstinspires.ftc.teamcode.Experimental.SubsystemTesters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Subsystem;

/**
 * Gamepad 1 -
 * A:           Intake In/Off
 * B:           Intake Out/Off
 * Back:        Intake Lock/Drop
 */

@TeleOp(name = "Subsystems: Intake Test")
public class TestIntake extends LinearOpMode {

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

        Intake.drop();

        while (opModeIsActive()) {

            Intake.appendTelemetry(true);
            telemetry.update();

            // INTAKE ==============================================================================

            // Release
            if (gamepad1.x && Constants.x == 0)
                Constants.x++;
            else if (!gamepad1.x && Constants.x == 1) {
                if (Intake.getPosition() == Intake.Position.LOCKED)
                    Intake.drop();
                else if (Intake.getPosition() == Intake.Position.DROPPED)
                    Intake.lock();
                Constants.x--;
            }

            // Rollers
            if (gamepad1.a && Constants.a == 0)
                Constants.a++;
            else if (!gamepad1.a && Constants.a == 1) {
                if (Intake.getDirection() == Intake.Direction.IN)
                    Intake.off();
                else
                    Intake.in();
                Constants.a--;
            } else if (gamepad1.b && Constants.b == 0)
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
        sleep(30000);
    }

    public void initialize(boolean isAuto) {

        this.isAuto = isAuto;

        telemetry.setAutoClear(false);

        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        sleep(500);

        Subsystem.initialize(hardwareMap,telemetry);

        Intake.initialize("rollers", "release");

        if (!isAuto) {
            telemetry.setAutoClear(true);
            while(!isStarted()) {
                Intake.appendTelemetry(true);
                telemetry.update();
            }
        }
    }
}