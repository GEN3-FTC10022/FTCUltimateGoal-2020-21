package org.firstinspires.ftc.teamcode.Experimental.SubsystemTesters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;

@TeleOp(name = "Subsystems: Intake Test")
public class TestIntake extends LinearOpMode {

    @Override
    public void runOpMode() {

        initialize(false);

        waitForStart();

        Intake.drop();

        doTeleOp();

        // doAuto();
    }

    public void doTeleOp() {
        while (opModeIsActive()) {

            // TELEMETRY ===========================================================================

            Intake.appendTelemetry(true);
            telemetry.update();

            // INTAKE ==============================================================================

            // Release
            if (gamepad1.x && Constants.x == 0)
                Constants.x++;
            else if (!gamepad1.x && Constants.x == 1) {
                if (Intake.getPosition() == Intake.Position.LOCKED)
                    Intake.drop();
                else
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

    }

    public void initialize(boolean isAuto) {

        telemetry.setAutoClear(false);

        Intake.initialize("rollers", "release");
        sleep(500);

        telemetry.setAutoClear(true);
    }
}