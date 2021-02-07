package org.firstinspires.ftc.teamcode.Quals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Quals.QualsSuperclass;

@Autonomous (name = "Quals: Auto")

public class QualsAuto extends QualsSuperclass {

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
        strafeRight(0.8,27);
        sleep(250);
        shootAll();
        shooter.setVelocity(0);

        if (vision.getStackHeight() == 0) { // Target Zone A
            strafeLeft(0.8, 25);
            sleep(500);
            forward(0.8, 12);
            sleep(500);
            rotateRight(0.5, 90);
            sleep(500);
            place();
            sleep(500);

            // Rotate to zero
            rotateLeft(0.5, 90);

        } else if (vision.getStackHeight() == 1) { // Target Zone B
            strafeLeft(0.8, 40);
            sleep(500);
            forward(0.8, 15);
            place();

        } else { // Target Zone C
            forward(0.8, 37);
            sleep(500);
            strafeLeft(0.8, 8);
            sleep(500);
            place();
            backward(0.8, 32);
            sleep(500);
        }

        sleep(30000);
    }
}