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
        Drivetrain.move(90,0.8,62);
        sleep(250);
        Drivetrain.move(0,0.35,20);
        sleep(250);

        // Shoot Rings
        sleep(4000);

        Shooter.stopLauncher();

        Intake.drop();

        if (Vision.ringsFound == 0) { // Target Zone A
            Drivetrain.move(180,0.5,25);
            sleep(250);
            Drivetrain.move(90,0.8,13.5);
            sleep(250);
            Drivetrain.rotate(0.5, -90.2);
            sleep(250);
            WobbleMech.place();

            // Face 2nd Wobble Goal
            Drivetrain.rotate(0.5, -96);
            sleep(250);
            Drivetrain.move(90,0.8, 48.5);
            WobbleMech.aim();
            sleep(1000);

            WobbleMech.collect();
            sleep(250);

            Drivetrain.rotate(0.5, 164);
            sleep(250);
            Drivetrain.move(90, 0.8, 20);
            sleep(250);

            WobbleMech.place();
            sleep(250);

            Drivetrain.rotate(0.5,108);
            sleep(250);
            Drivetrain.move(0,0.5,24);
            sleep(250);

        } else if (Vision.ringsFound == 1) { // Target Zone B

            Drivetrain.move(180,0.5,35);
            sleep(250);
            Drivetrain.move(90,0.8,6);
            WobbleMech.place();

            // Face left
            Drivetrain.rotate(0.5, -185);
            sleep(250);

            Drivetrain.move(90,0.8, 40);
            WobbleMech.aim();
            sleep(1000);

            WobbleMech.collect();
            sleep(250);

            Drivetrain.rotate(0.5, 185);
            sleep(250);
            Drivetrain.move(90,0.8, 42);
            sleep(250);

            WobbleMech.place();
            sleep(250);


        } else { // Target Zone C
            Shooter.runLauncher();
            Drivetrain.move(90,0.8,37);
            sleep(500);
            Drivetrain.move(180,0.5,11);
            sleep(500);
            WobbleMech.place();
            Drivetrain.move(270,0.8,27);
            sleep(500);

            // Face left
            Drivetrain.rotate(0.5, 89);
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