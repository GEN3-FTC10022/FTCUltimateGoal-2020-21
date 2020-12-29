package org.firstinspires.ftc.teamcode.Quals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Quals.QualsSuperclass;

@Autonomous (name = "Park")

public class Park extends QualsSuperclass {

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();
        strafeLeft(0.3,64);
        sleep(500);

        rotateRight(0.3, 90);
        sleep(500);
        wobbleMech.setMotorPosition(2);
        sleep(500);
        wobbleMech.open();
        sleep(500);
        wobbleMech.setMotorPosition(0);
    }

}