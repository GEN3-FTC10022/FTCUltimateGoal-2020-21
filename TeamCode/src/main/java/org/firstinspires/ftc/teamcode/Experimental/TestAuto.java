package org.firstinspires.ftc.teamcode.Experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;

@Autonomous (name = "Test: Auto")

public class TestAuto extends TestSuperclass {

    @Override
    public void runOpMode() {

        initialize(true);

        waitForStart();

        Vision.scanStack(false, false);

        Shooter.setTarget(4);
        Shooter.runLauncher();

        // Move to launch line
        forward(0.8, 57);
        sleep(250);
        strafeRight(0.8,27);
        sleep(250);
        Shooter.shootAll();
        Shooter.setTarget(0);

        if (Vision.ringsFound == 0) { // Target Zone A
            strafeLeft(0.8, 25);
            sleep(500);
            forward(0.8, 12);
            sleep(500);
            rotateRight(0.5, 90);
            sleep(500);
            WobbleMech.place();

            // Rotate to zero
            rotateLeft(0.5, 90);

        } else if (Vision.ringsFound == 1) { // Target Zone B
            strafeLeft(0.8, 40);
            sleep(500);
            forward(0.8, 15);
            WobbleMech.place();

        } else { // Target Zone C
            forward(0.8, 37);
            sleep(500);
            strafeLeft(0.8, 8);
            sleep(500);
            WobbleMech.place();
            backward(0.8, 32);
            sleep(500);
        }

        sleep(30000);
    }
}