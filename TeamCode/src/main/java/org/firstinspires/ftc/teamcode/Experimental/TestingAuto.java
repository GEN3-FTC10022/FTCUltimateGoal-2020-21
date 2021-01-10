package org.firstinspires.ftc.teamcode.Experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Quals.QualsSuperclass;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleMech;

@Autonomous (name = "Auto: Test")

public class TestingAuto extends TestingSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();
    }

}