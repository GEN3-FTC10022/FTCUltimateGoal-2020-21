package org.firstinspires.ftc.teamcode.Quals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Quals.QualsSuperclass;

@Autonomous (name = "Auto: Qual 1")
/*
 * This is a red side autonomous, but since there is only one side
 * for now, it's just labelled "auto."
 *
 * I believe our field is blue color, but red set up (remote field set up, per the game manual).
 */
public class QualsAuto extends QualsSuperclass {

    @Override
    public void runOpMode() {

        initialize();
    }

}