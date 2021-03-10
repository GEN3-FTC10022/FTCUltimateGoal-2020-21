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
        Drivetrain.move(90,62);
        sleep(250);
        Drivetrain.move(0,0.35,20);
        sleep(250);

        // Launch rings to high goal
        Shooter.launchAll(2);
        sleep(250);

        Intake.drop();

        if (Vision.ringsFound == 0) { // Target Zone A

            // Place 1st Wobble Goal
            Drivetrain.move(180,25);
            sleep(250);
            Drivetrain.move(90,13.5);
            sleep(250);
            Drivetrain.rotate(-90.2);
            sleep(250);
            WobbleMech.place();
            sleep(250);

            // Collect 2nd Wobble Goal
            Drivetrain.rotate(-96);
            sleep(250);
            Drivetrain.move(90,48.5);
            WobbleMech.aim();
            sleep(1000); // Wait till move is finished before collecting
            WobbleMech.collect();
            sleep(250);

            // Place 2nd Wobble Goal
            Drivetrain.rotate(164);
            sleep(250);
            Drivetrain.move(90,20);
            sleep(250);
            WobbleMech.place();
            sleep(250);

            // Park at field-centric zero heading
            Drivetrain.rotateTo(-90);
            sleep(250);
            Drivetrain.move(0,24);
            sleep(250);

        } else if (Vision.ringsFound == 1) { // Target Zone B

            // Place 1st Wobble Goal
            Drivetrain.move(180,35);
            sleep(250);
            Drivetrain.move(90,6);
            sleep(250);
            WobbleMech.place();
            sleep(250);

            // Collect 2nd Wobble Goal
            Drivetrain.rotate(-185);
            sleep(250);
            Drivetrain.move(90,40);
            WobbleMech.aim();
            sleep(1000);
            WobbleMech.collect();
            sleep(250);

            // Place 2nd Wobble Goal
            Drivetrain.rotate(0.5, 185);
            sleep(250);
            Drivetrain.move(90,0.8, 42);
            sleep(250);
            WobbleMech.place();
            sleep(250);

            // Park at field-centric zero heading
            Drivetrain.rotateTo(-90);


        } else { // Target Zone C

            // Place 1st Wobble Goal
            Drivetrain.move(90,37);
            sleep(250);
            Drivetrain.move(180,11);
            sleep(250);
            WobbleMech.place();
            sleep(250);

            // Park at field-centric zero heading
            Drivetrain.move(270,0.8,27);
            sleep(250);
            Drivetrain.rotateTo(-90);
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