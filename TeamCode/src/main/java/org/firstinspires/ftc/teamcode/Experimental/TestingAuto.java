package org.firstinspires.ftc.teamcode.Experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Test: Auto")

public class TestingAuto extends TestingSuperclass {

    @Override
    public void runOpMode() {

        initialize(true);

        waitForStart();

        vuforiaScanStack(false, false);

        shooter.setTargetVelocity(3);
        shooter.runShooter();

        // Move to launch line
        forward(0.8, 57);
        sleep(250);
        strafeRight(0.8,20);
        sleep(250);
        shootAll();
        shooter.setVelocity(0);

        if (vision.stackHeight == 0) { // Target Zone A
            strafeLeft(0.8, 10);
            sleep(500);
            rotateRight(0.5, 90);
            // rotate(0.5,-90);
            // rotateToAnglePID(0.5, -90);
            sleep(500);
            place();
            sleep(500);

            // Rotate to zero
            rotateLeft(0.5, 90);
            // rotate(0.5, 90);
            // rotateToAnglePID(0.5, 0);

        } else if (vision.getStackHeight() == 1) { // Target Zone B
            strafeLeft(0.8, 27);
            sleep(500);
            place();

        } else { // Target Zone C
            forward(0.8, 45);
            sleep(500);
            strafeLeft(0.8, 6);
            sleep(500);
            place();
            backward(0.8, 40);
            sleep(500);
        }
    }
}