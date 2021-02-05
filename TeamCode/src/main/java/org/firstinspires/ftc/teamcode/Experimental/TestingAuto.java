package org.firstinspires.ftc.teamcode.Experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Test: Auto")

public class TestingAuto extends TestingSuperclass {

    @Override
    public void runOpMode() {

        initialize(true);

        waitForStart();

        // vuforiaScanStack(false, false);

        shooter.setTargetVelocity(3);
        shooter.runShooter();

        // Move to launch line
        forward(0.8, 57);
        sleep(250);
        rotateToAngle(0.8,0,true);
        sleep(250);
        strafeRight(0.8,20);
        sleep(250);

        shootAll();

        /*
        if (vision.stackHeight == 0) {
            forward(0.5, 50);
            sleep(500);
        } else if (vision.getStackHeight() == 1) {
            forward(0.5, 35);
            sleep(500);
            rotateLeft(0.5,45);
            sleep(500);
            forward(0.5,5);
            sleep(500);
        } else {
            forward(0.5, 50);
            sleep(500);
        }

        // Drop
        drop();

        if (vision.getStackHeight() == 0) {
            forward(0.5, 35);
            sleep(500);
        } else if (vision.getStackHeight() == 1) {
            forward(0.5, 35);
            sleep(500);
            rotateLeft(0.5,45);
            sleep(500);
            forward(0.5,5);
            sleep(500);
        } else {
            forward(0.5, 50);
            sleep(500);
        }
         */

        sleep(30000);

        stop();
    }
}