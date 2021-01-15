package org.firstinspires.ftc.teamcode.Quals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Quals.QualsSuperclass;

@Autonomous (name = "Auto: Qual 2")

public class QualsAuto extends QualsSuperclass {

    @Override
    public void runOpMode() {

        initialize(true);

        waitForStart();

        rotateToAngle(0.3,-180);

        sleep(2000);

        // rotateToAngle(0.3,45);

        // sleep(2000);

        telemetry.addData("Final Angle", drivetrain.getHeading(false));
        telemetry.update();

        sleep(30000);

        stop();
    }

}