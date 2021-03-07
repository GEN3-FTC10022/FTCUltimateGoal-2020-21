package org.firstinspires.ftc.teamcode.Experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Subsystem;

@Autonomous (name = "Test: Auto")

public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();

        sleep(4000);
        telemetry.addData("Velocity", Shooter.getVelocity());
        telemetry.update();
        sleep(2000);
        Shooter.stopLauncher();

        sleep(30000);
    }

    private void initialize() {

        Constants.reset();

        telemetry.setAutoClear(false);

        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        Subsystem.initialize(hardwareMap, telemetry);
        Vision.initialize();
        WobbleMech.initialize();
        Intake.initialize();
        Shooter.initialize();
        Drivetrain.initialize(true);

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
    }
}