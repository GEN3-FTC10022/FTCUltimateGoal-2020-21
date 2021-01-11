package org.firstinspires.ftc.teamcode.Quals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Quals.QualsSuperclass;

@Autonomous (name = "Auto: Qual 2")

public class QualsAuto extends QualsSuperclass {

    @Override
    public void runOpMode() {

        initialize(true);

        waitForStart();

        vuforiaScanStack(false,false);

        sleep(60000);

        stop();
    }

}