package org.firstinspires.ftc.teamcode.Experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Quals.QualsSuperclass;

@Autonomous (name = "Test: Auto")

public class TestingAuto extends TestingSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();
    }

}