package org.firstinspires.ftc.teamcode.Quals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Quals.QualsSuperclass;

@Autonomous (name = "Auto: Qual 3")

public class QualsAuto extends QualsSuperclass {

    @Override
    public void runOpMode() {

        initialize(true);

        waitForStart();

        vuforiaScanStack(false, false);

        // Move to zone
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

        /*
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