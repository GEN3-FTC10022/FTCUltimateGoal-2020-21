package org.firstinspires.ftc.teamcode.Experimental.SubsystemTesters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;

@TeleOp(name = "Subsystems: Intake Test")
public class TestIntake extends LinearOpMode {

    private Constants constants = new Constants();
    private Intake intake = new Intake();

    @Override
    public void runOpMode() {

        initialize(false);

        telemetry.setAutoClear(true);

        waitForStart();

        intake.down();

        doTeleOp();

        // doAuto();
    }

    public void doTeleOp() {
        while (opModeIsActive()) {

            // TELEMETRY ===========================================================================

            displayTeleOpTelemetry();

            // INTAKE ==============================================================================

            // Release
            if (gamepad1.x && constants.x == 0)
                constants.x++;
            else if (!gamepad1.x && constants.x == 1) {
                if (intake.position == Intake.Position.UP)
                    intake.down();
                else
                    intake.up();
                constants.x--;
            }

            // Rollers
            if (gamepad1.a && constants.a == 0)
                constants.a++;
            else if (!gamepad1.a && constants.a == 1) {
                if (intake.status == Intake.Status.IN)
                    intake.off();
                else
                    intake.in();
                constants.a--;
            } else if (gamepad1.b && constants.b == 0)
                constants.b++;
            else if (!gamepad1.b && constants.b == 1) {
                if (intake.status == Intake.Status.OUT)
                    intake.off();
                else
                    intake.out();
                constants.b--;
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

        // Intake ==================================================================================
        intake.roller = (DcMotorEx)hardwareMap.dcMotor.get("rollers");
        intake.release = hardwareMap.servo.get("release");
        intake.initialize();
        telemetry.addLine("Intake initialized");
        telemetry.update();
        sleep(500);
    }

    public void displayTeleOpTelemetry() {

        telemetry.addLine("=== INTAKE ===");
        telemetry.addData("Rollers", intake.status);
        telemetry.addData("Position", intake.position);
        telemetry.addLine();
    }


}