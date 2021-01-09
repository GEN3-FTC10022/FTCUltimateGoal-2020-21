package org.firstinspires.ftc.teamcode.Quals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Quals.QualsSuperclass;

@Autonomous (name = "Auto: Park")

public class Park extends QualsSuperclass {

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();
    }

}