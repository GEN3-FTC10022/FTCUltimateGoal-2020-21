package org.firstinspires.ftc.teamcode.Quals;

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

@Autonomous (name = "Quals: Auto")

public class QualsAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();

        Vision.scanStack(false, false);

        // Move to launch line
        Drivetrain.move(90,0.8,58.5);
        sleep(250);
        Drivetrain.move(0,0.8,15);
        sleep(250);
        Shooter.runLauncher();
        sleep(4000);
        Shooter.shootAll();
        Shooter.setTarget(0);

        if (Vision.ringsFound == 0) { // Target Zone A
            Shooter.runLauncher();
            Drivetrain.move(180,0.8,25);
            sleep(500);
            Drivetrain.move(90,0.8,12);
            sleep(500);
            Drivetrain.rotate(0.5, -90);
            sleep(500);
            WobbleMech.place();

            // Rotate to zero
            Drivetrain.rotate(0.5, 90);

        } else if (Vision.ringsFound == 1) { // Target Zone B
            Shooter.runLauncher();
            Drivetrain.move(180,0.8,35);
            sleep(500);
            Drivetrain.move(90,0.8,15);
            WobbleMech.place();

        } else { // Target Zone C
            Shooter.runLauncher();
            Drivetrain.move(90,0.8,37);
            sleep(500);
            Drivetrain.move(180,0.8,8);
            sleep(500);
            WobbleMech.place();
            Drivetrain.move(270,0.8,32);
            sleep(500);
        }

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