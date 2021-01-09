package org.firstinspires.ftc.teamcode.Experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Quals.QualsSuperclass;

@Autonomous (name = "Auto: Test")

public class TestingAuto extends TestingSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();

        sleep(60000);

        stop();
    }

}